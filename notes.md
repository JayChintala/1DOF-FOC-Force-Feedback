# Notes: autonomy, 2-DoF redesign, and host/Pi setup

Discussion with Claude, 2026-11-17 to 2026-11-18. Prices and part details are
estimates as of then; check them before ordering.

---

## 1. The problem being solved

Sensorless force-feedback teleoperation. A leader motor (held by a person)
drives a follower motor. The follower's contact force is estimated from FOC
motor current, with no force sensor, and rendered back on the leader so the
hand feels what the follower touches. Framed in the deck as
"Force-Aware Telepresence for Robot Learning".

- Hardware today: two B-G431B-ESC1 boards on CAN, a Raspberry Pi 4 with a
  Waveshare MCP2518FD CAN HAT, a Windows host PC (JayPC) for firmware.
- Core finding that drove Phase C: in the baseline, the felt force was an
  echo of the operator's own push (iq_b_cmd = -tau_h), not the environment.
- State on 2026-11-15 (see `ESC_based/software/TEST_LOG.md`, TL;DR and
  "0c. RESUME HERE"): Phases A to C largely done; tip force calibrated at
  1.058 N/A, linear to 0.8 A (Tafter 400); follower separates rigid from
  foam (9.7x stiffness); symmetric PEER mode explored but blocked by a
  ~57 Hz resonance. Next step: D1 (force accuracy on the scale).

---

## 2. What would let Claude test autonomously (1-DoF rig)

Today the person is the input to every test (presses the leader) and the
output (reads the scale, judges "feel", flashes firmware, re-plugs ESCs).
Each needs a stand-in.

Priority order:

1. **Robot "operator" on the leader + torque sensor.** A third torque-controlled
   motor plays the hand (sweeps, presses, taps, holds, adjustable hand-like
   stiffness/damping). An inline reaction torque sensor (about 0.2-0.5 N·m
   range; the 0.8 A ceiling is about 52 mN·m at the shaft) gives ground truth
   for what the hand feels, turning "light and responsive" into numbers.
   Use a compliant coupling, since a rigid one hides instabilities a real hand
   would trigger. Cheaper version: load cell at the leader arm tip, linked to
   the operator arm.
2. **Load cell at the follower's contact surface.** Replaces reading grams off
   the scale. Button or S-beam cell, a few N. Needs a fast ADC (ADS1220 or
   ADS131M02, 1-2 kSPS+); HX711 at 10-80 Hz is too slow for contact
   transients. Optionally a servo turret / stage to switch environments
   (rigid, foam, spring, none) automatically.
   Items 1 + 2 together directly measure the actual goal: does leader torque
   track the follower's contact force, or just echo the push?
3. **Firmware build and flash without the person.** (Resolved differently,
   see section 6: build/flash on JayPC, drive the Pi over SSH.)
4. **Power the computer can control and measure.** Programmable bench supply
   (Riden RD60xx over Modbus, Korad over serial) or a relay/MOSFET on ESC
   power, so dropouts don't need a re-plug. Also logs bus voltage/current.
   Fix the suspect ESC power connectors.
5. **Safety for unattended runs.** Hardware E-stop within reach; temperature
   sensor (thermistor / DS18B20) on each motor case (0.8 A is the thermal
   rating); mechanical hard stops; firmware current and CAN-timeout limits.
6. **Data quality.** Clamp the motor mounts (B7 flagged wobble). Go back to the
   gs_usb/candleLight adapter (0 dropped frames vs the HAT's 1.7%) or rely
   on the firmware µs timestamps. Optional Pi camera; accelerometers on the
   arm tips for the 57 Hz resonance and wobble.

**What this can't replace:** a robot operator is not a human hand. A
controller stable against a synthetic hand may still feel wrong or go
unstable in a real one. Realistic split: Claude iterates against measured
targets, then hands over a short list of candidates for ~10-minute hand tests.

**Minimum set:** items 1-3 plus the E-stop.

---

## 3. 2-DoF redesign

### Motor controllers: mjbots moteus + pi3hat

The B-G431B-ESC1 + MCSDK stack is behind much of the pain so far (7/rev
ripple, Tafter current breakdown, hand-written CAN, dropouts).

- **moteus** (r4.11 or n1): open-source firmware; FOC with torque, position
  and feedforward commands over CAN-FD; on-board 14-bit magnetic encoder plus
  an aux encoder port. **Firmware flashes over CAN** (`moteus_tool --flash`),
  no programmer. Most work needs no custom firmware: send position,
  stiffness, damping and torque targets at kHz rates.
- moteus runs its own stiffness/damping terms on the controller at 40 kHz,
  so local damping has no bus delay. This is the problem that pushed the
  1-DoF law onto the ESCs; here the Pi only sends setpoints and feedforward
  at 1 kHz.
- **pi3hat**: 5 CAN-FD ports over low-latency SPI, plus an IMU. Replaces the
  Waveshare HAT that drops frames.
- Control loop in C++ on the Pi, on a dedicated (isolated) CPU core,
  1-2 kHz for all 4 motors.

### Mechanism: planar five-bar (pantograph) with capstan drives

- Standard 2-DoF haptic layout (Hayward's Pantograph, Haply).
- Both motors fixed at the base: moving mass is only light links, which is
  what makes it transparent.
- Stiff parallel structure, closed-form kinematics; hand force maps to motor
  torques through F = J^-T τ. Keep the workspace away from singularities.
- **Capstan drive**: steel or Dyneema cable on a small motor pulley driving a
  sector pulley on each link. Zero backlash, very low friction, fully
  backdrivable, and multiplies encoder resolution at the joint.
- Rejected: serial 2R arm (carries or belts the elbow motor), XY gantry
  (rail friction, heavy moving mass).

**Rough sizing** (10 x 10 cm workspace): ~15 cm links; ~5 N continuous and
~15 N peak at the tip (desktop haptic devices give ~3 N). Through a 10:1
capstan that is ~0.1 N·m continuous, ~0.35 N·m peak per motor.

### Motors and encoders

- **Motors:** 5208-6374 class outrunner (mj5208 or a similar gimbal-style
  motor). Choose **low cogging over high current rating**: cogging was the
  biggest transparency problem, and the capstan removes the need for big
  currents. Premium option: slotless motors (Maxon ECX / EC-i), near-zero
  cogging, costlier, compatibility with moteus to be checked.
- **Encoders:** moteus on-board encoder per motor (~160k counts/rev at the
  joint through a 10:1 capstan). Optional absolute encoder on each joint
  output (RLS AksIM-2, supported by moteus) to measure cable stretch.

### Peer-to-peer (either arm can lead)

Mostly a control-law choice (symmetric bilateral law), but it shapes the
hardware:

- **The two arms must be identical**, including kinematics and calibration.
- **Each tip is both handle and tool.** One force sensor per tip serves both
  roles (hand force when held, contact force when touching). 2 sensors total.
- **Felt mass is both arms combined.** The 1-DoF PEER test felt 3.17x heavier
  than teleop. Reflected motor inertia scales with gear ratio squared, so
  use a **6-8:1 capstan** instead of 10:1, and light carbon-tube links.
- **Damping is the open risk.** 1-DoF PEER was blocked by an undamped ~57 Hz
  resonance. On-controller damping at 40 kHz should handle it; test it first.

### Why "three units", and why it's optional

- The bilateral system itself is **two identical arms**.
- A third unit would only be a robot stand-in for the hand (coupled through a
  compliant joint + force sensor), so interaction tests can run unattended.
  In peer mode it must be able to attach to either arm.
- **Build two first.** Add the third only if hand-testing time becomes the
  bottleneck. Without it Claude can still do calibration, kinematics,
  friction/cogging maps, plant ID, and free-space and contact tests against
  fixed environments.

### Ground-truth sensors (validation only, not in the control loop)

- Force sensor at each arm tip: ATI Nano17 / Bota (commercial 6-axis), or a
  DIY 2-axis flexure with load cells and a fast ADC.
- Swappable environments: rigid wall, foam, spring, free space.
- Safety: hardware E-stop, motor temperature sensors.

### Phasing

1. Build one five-bar. Claude brings up moteus, calibrates, verifies
   kinematics, maps friction and cogging, identifies the plant.
2. Build the second. Port the bilateral controller and sensorless force
   estimator to 2-DoF; test against the tip force sensors.
3. (Optional) Add the operator unit for unattended sweeps and tuning; final
   hand tests by the user.

---

## 4. Budget (estimates)

Confirmed at the time: moteus r4.11 ~$79 each (after an mjbots price cut),
moteus n1 $159. Unconfirmed guesses: pi3hat, motors, commercial F/T sensors.

| Configuration | Approx. total |
|---|---|
| Two identical arms (peer-to-peer), 3D-printed parts, no ground-truth sensors | ~$1.5k |
| + DIY 2-axis force sensor at each tip (load cells + fast ADC) | ~$1.8-2k |
| + third "operator" unit for unattended testing | ~$2.5k |
| Sky-is-the-limit: machined aluminum, absolute encoder on every joint, commercial 6-axis F/T sensors | ~$8-15k+, mostly the F/T sensors |

Per arm (2 motors), ~$550: 2x moteus r4.11 (~$160), 2x 5208-class motors
(~$150-200), capstan parts and printed links (~$150-250).

Shared, ~$400: pi3hat (~$150), 24 V supply, ideally controllable
(~$50-150), E-stop / connectors / CAN wiring (~$100).

Machined aluminum instead of printed parts: roughly +$500-1,500 per arm.
Worth it later for stiffness, not for the first build.

---

## 5. Is a Pi 4 enough?

Yes, and it is the right board. The pi3hat targets the Pi 4 and 3B+; Pi 5
support is unconfirmed and one retailer lists it as not supported. The
current 4 GB Pi 4 can run a 1 kHz C++ loop for 4 motors on an isolated core;
mjbots runs 12-motor robots on the same Pi 4 + pi3hat.

Caveat: building custom moteus firmware may need an x86 Linux machine (to
verify). Custom firmware isn't expected to be needed.

---

## 6. Host PC + Pi setup (done 2026-11-18)

**Architecture:** run Claude Code on the host PC (JayPC, Windows), which has
the full firmware tree, CubeIDE and both ST-LINK USBs. Build and flash there;
run CAN tests on the Pi over SSH; copy logs back. SSH latency doesn't matter:
the control loop runs on the Pi, SSH only launches it.

```
JayPC (Claude Code runs here)
 ├─ build firmware        CubeIDE headless build
 ├─ flash ESC1 + ESC2     STM32_Programmer_CLI, each board by ST-LINK serial
 └─ ssh pi "..."  ──────► Pi: CAN tests, teleop, logging
    scp pi:logs ◄──────── logs back for analysis
```

**Done:**

- Passwordless SSH from JayPC to the Pi (192.168.1.164, user `jayrpi4`):
  JayPC's key added to the Pi's `~/.ssh/authorized_keys` (backup at
  `authorized_keys.bak`). Optional `Host pi` entry in
  `C:\Users\jayis\.ssh\config`.
- `STM32_Programmer_CLI` (STM32CubeProgrammer v2.22.0) on JayPC's user PATH:
  `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin`.
  (CubeIDE 2.1.1 also bundles a copy.)
- **Board mapping, verified:**

  | Board | CAN node | Firmware build | ST-LINK serial | COM |
  |---|---|---|---|---|
  | ESC1 | base 0x000 | `CAN_NODE_ID 0` | `066BFF525266555067073237` | COM3 |
  | ESC2 | base 0x020 | `CAN_NODE_ID 1` | `066BFF515266555067014734` | COM6 |

  `CAN_NODE_ID` is compile-time, so each board needs its own build; flashing
  the wrong one gives two nodes the same ID.
  Flash: `STM32_Programmer_CLI -c port=SWD sn=<SN> -w <elf> -v -rst`.

**Lessons from identifying the boards:**

- Watching for a silent gap doesn't work: the boards reboot in under 3 ms,
  and the CAN HAT's dropped frames cause 3-7 ms gaps on both nodes anyway.
- `-halt` through the CLI does not keep the core halted.
- What works: the TELEM µs timestamp (bytes 6-7). Consecutive steps are
  multiples of 1000 µs within ±6 µs; a reset gives an off-grid step.
- A CLI reset leaves the board in its uncalibrated idle state (TELEM Iq reads
  ~±25-28 A until START); the normal START handles it.

**Still to do:**

1. Confirm CubeIDE's headless build works on JayPC (needs the workspace and
   project paths).
2. Open Claude Code on JayPC in this repo. Claude's memory notes from the Pi
   are copied to `ESC_based/docs/claude_memory/`.
3. Not decided yet for the 2-DoF BOM: fabrication options (3D printing only,
   or machining / laser cutting), workspace and force targets if different
   from section 3.

---

## Sources (prices and compatibility, checked 2026-11-18)

- mjbots product and price-cut posts: https://blog-575.pages.dev/categories/products
- moteus r4.11 (reseller): https://reach.dog/shop/mjbots-robotic-systems/products/moteus-r4-11
- moteus r4.11 developer kit: https://reach.dog/shop/mjbots-robotic-systems/products/moteus-r4-11-developer-kit
- pi3hat r4.5 listing (Pi 4/3B+, not 5): https://reach.dog/shop/mjbots-robotic-systems/products/mjbots-pi3hat-r4-5
- pi3hat on GitHub: https://gh.r-mj.com/mjbots/pi3hat
- moteus documentation: https://mjbots.github.io/moteus
- Bota MiniONE: https://casscale.net/products/bota-systems-minione-6-axis-force-torque-sensor/?lang=en

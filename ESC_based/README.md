# ESC_based — coupling on the ESCs

The bilateral coupling law (spring + damper between the two shafts) runs **on the ESCs**, not on the Pi. Each ESC reads the other one's TELEM frame directly off the CAN bus, so the Pi is out of the loop. The Pi sets the gains, starts the motors, asks for a mode, and logs what happens.

The previous design, where the Pi computes everything and sends `SET_IQ`, is tagged **`pre-esc-based`**. It still lives unchanged in `../STM32` and `../software`.

```
ESC_based/
  STM32/MCWorkbench/            copy of the firmware project (CubeIDE project name: ESC_based)
    Inc/can_driver.h            CAN protocol: IDs, payloads, peer addressing
    Inc/couple_ctrl.h           modes, the law, every default and limit
    STM32CubeIDE/Application/User/can_driver.c    transport: filters, peer RX, TELEM + DBG
    STM32CubeIDE/Application/User/couple_ctrl.c   owns the Iq reference: direct SET_IQ or the law
  software/
    coupling_monitor.py         engage + log (terminal readout, CSV, PNG)
    plot_coupling.py            re-plot a log
    can_interface.py            ../software/can_interface.py + coupling commands + DBG
```

## The law

Each ESC runs this every 1 kHz tick, with "own" and "peer" swapped on the other ESC:

```
err  = (own - own0) - (peer - peer0)                      counts (4000 per rev)
derr = own_vel - peer_vel                                 counts/s
iq   = clamp(-(Kp*err + Kd*derr + Kd_local*own_vel), ±Iq_max)
```

With `Kd_local = 0` this is the same law as `position_mirror_test.py`, in the same units, so gains carry over directly. `Kd_local` damps the motor's own velocity. It is the only term with no bus delay, which is what should let Kp go much higher than the Pi could run it. The cost is a drag you can feel when moving the shaft freely.

`own0` and `peer0` are captured when the mode engages. A mode engages only once **both shafts are still** (under 2000 counts/s), so both ESCs capture their zero from the same physical state. Keep your hands off the shafts when engaging.

## Build and flash

1. In CubeIDE: **File → Import → Existing Projects into Workspace** → `ESC_based/STM32/MCWorkbench/STM32CubeIDE`. The project is named `ESC_based`, so it can sit next to the original `MCWorkbench` project in the same workspace.
2. Build configuration **Debug** builds ESC 1. **Debug_ESC2** builds ESC 2 (it adds `CAN_NODE_ID=1`). Build **both once** first, using the arrow next to the hammer icon. CubeIDE greys out a launch config until its `.elf` exists, and the copy ships without build output.
3. Flash with the launch configs **ESC_based ESC1** and **ESC_based ESC2**. They keep the original ST-Link serial numbers, so each one finds its own board.

Both ESCs must run this firmware. The monitor refuses to engage if a board sends TELEM but no DBG, which means that board is still on the old firmware.

## Bring-up, in order

Run everything from `ESC_based/software/` on the Pi, venv active, with can0 up at 1 Mbit/s as before.

**1. Direct mode still works.** At boot both ESCs are in direct `SET_IQ` mode, the same protocol as before, so the old scripts in `../software` work as they are. One difference: **`SET_IQ` must be resent at least every 200 ms or the ESC zeroes the current** (`COUPLE_DIRECT_TIMEOUT_US`). Control loops already resend far more often than that. `spin_motor2.py` and `sign_check_test.py` send one command and then sleep, so they will stop after 200 ms.

**2. Stiff hold, one motor.** Each ESC holds its own position, with no peer involved:

```
python3 coupling_monitor.py --mode hold --motors a --kp 0.001
```

Raise `--kp` step by step and push the shaft each time. Note where it starts to buzz or ring, then try adding `--kd-local`. This finds the stiffness a single motor can manage before bus delay is involved.

**3. Bilateral coupling.**

```
python3 coupling_monitor.py --kp 0.001 --kd 0.00003
```

If the pair buzzes with nobody holding the sticks at a Kp one motor holds quietly, that's the 1–2 ms bus delay. Add `--predict`, which makes each ESC couple to where the other shaft is *now* (its last position plus velocity × age), and/or a little `--kd-local`.

Turn one shaft and the other follows. Clamp one and the other locks up, up to the force `Iq_max` gives. `--duration 0` runs until Ctrl+C. Each run writes `logs/coupling_<mode>_KP…csv`, a `.json` with the settings, and a `.png`.

Once you have settled values, put them in as `COUPLE_DEFAULT_*` in `couple_ctrl.h`. The ESCs then start up with them.

## Protocol

| ID | Direction | Payload |
|---|---|---|
| base+0x001 START | Pi→ESC | — |
| base+0x002 STOP | Pi→ESC | — (also cancels coupling) |
| base+0x003 SET_IQ | Pi→ESC | float32 A. Direct mode only; resend within 200 ms |
| base+0x004 COUPLE_GAINS | Pi→ESC | float32 Kp [A/count], float32 Kd [A/(count/s)] |
| base+0x005 COUPLE_LOCAL | Pi→ESC | float32 Kd_local [A/(count/s)], float32 Iq_max [A, capped at 0.8] |
| base+0x006 COUPLE_MODE | Pi→ESC | uint8: 0 off, 1 hold, 2 peer. Sending it again re-zeroes |
| base+0x012 TELEM | ESC→all | unchanged: float32 Iq, uint16 encoder, uint16 µs clock |
| 0x100+base+0x012 DBG | ESC→Pi | int16 Iq cmd [mA], int16 err [counts], uint16 µs clock (same as TELEM), uint8 status, uint8 peer age [0.1 ms] |

base = 0x000 for ESC 1 and 0x020 for ESC 2. DBG is sent every tick while coupling is requested or engaged, immediately when its status byte changes, and at 10 Hz otherwise. Its IDs are above 0x100 so DBG always loses arbitration to control traffic.

Status bits: 0 RUN, 1 ENGAGED, 2 REQUESTED, 3 MODE_PEER, 4 SATURATED, 5–7 trip code.

## Safety behaviour

Each of these drops coupling, zeroes Iq and latches a trip code that the monitor prints:

| Trip | Cause |
|---|---|
| 1 peer timeout | peer TELEM older than 5 ms: the other ESC is dead, unpowered, or the bus is down |
| 2 error limit | \|err\| > 40000 counts (10 rev) |
| 3 velocity limit | own shaft > 200000 counts/s (50 rev/s); a wrong torque sign shows up as this or as 2 |
| 4 host timeout | direct mode: no `SET_IQ` for 200 ms |
| 5 left RUN | the motor faulted while engaged |

A trip on one ESC does not trip the other. If ESC A trips, ESC B stays engaged and holds toward A's now-passive position until the Pi sends OFF or STOP, which the monitor always does on exit.

## Also changed compared to `pre-esc-based`

- **References apply 1 ms sooner.** MCSDK normally applies a new current reference at the start of the *next* medium-frequency tick. Both direct `SET_IQ` and the law now apply theirs immediately (`CommandIq()` in `couple_ctrl.c`).
- **Timestamp bug fixed.** The uint16 µs clock in TELEM used to jump back about 32 ms every 25.26 s, whenever the 32-bit cycle counter wrapped. Any velocity differenced across that sample got a bogus dt. The original firmware in `../STM32` still has this bug.
- **Filter count 2.** `FDCAN1.StdFiltersNbr` is 2 in the `.ioc` and `main.c`. `CAN_Driver_Init()` also forces it to 2 if a regeneration ever puts 1 back. With 1, the peer filter would silently never match.

## Regeneration caveats

- The `mc_app_hooks.c` include caveat from `CAN_setup_reference.md` still applies. The hook body is now `CAN_ProcessPendingMessages(); CAN_ControlTick();`.
- Workbench/CubeMX regeneration may reset the Eclipse project name in `.project`/`.cproject` back to `MCWorkbench`. If that happens, set it to `ESC_based` again, or the launch configs won't find the project.
- `couple_ctrl.c` is picked up automatically, like `can_driver.c`: CubeIDE compiles every `.c` in `STM32CubeIDE/Application/User/`.

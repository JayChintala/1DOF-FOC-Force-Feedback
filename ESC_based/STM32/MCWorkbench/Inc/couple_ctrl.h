/*
 * couple_ctrl.h
 *
 * Owns this ESC's Iq reference. Exactly one source drives it at a time:
 *
 *   COUPLE_MODE_OFF   direct: the Pi's SET_IQ, as in the original firmware,
 *                     now with a host-silence timeout.
 *   COUPLE_MODE_HOLD  spring+damper to where THIS shaft was when the mode
 *                     engaged. Single-motor bring-up for stiff gains.
 *   COUPLE_MODE_PEER  spring+damper to the OTHER ESC's shaft, read straight
 *                     off the bus. The Pi is not in this loop.
 *   COUPLE_MODE_P2P   peer-to-peer bilateral control with disturbance
 *                     observers (Keio-style acceleration-based 4-channel
 *                     control). Same transport as PEER, different law --
 *                     see "P2P mode" below.
 *
 * The law, run once per 1 kHz tick on each ESC:
 *
 *   err  = (own - own0) - (peer - peer0)        counts   (HOLD: no peer terms)
 *   derr = own_vel - peer_vel                   counts/s
 *   iq   = clamp(-SIGN*(Kp*err + Kd*derr + Kd_local*own_vel), +-Iq_max)
 *
 * Both ESCs run it with "own" and "peer" swapped, so err_1 = -err_2 and the
 * two currents are equal and opposite. With Kd_local = 0 that is exactly
 * position_mirror_test.py's law, in the same units, so gains carry over.
 *
 * Kd_local damps this shaft's own velocity. It is the only term with no
 * transport delay at all -- which is what should let Kp go far higher than
 * the Pi could ever run it -- but it also adds drag you feel in free motion.
 */

#ifndef COUPLE_CTRL_H
#define COUPLE_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "can_driver.h"

/* ---- Modes (COUPLE_MODE payload byte) ---- */
#define COUPLE_MODE_OFF 0u
#define COUPLE_MODE_HOLD 1u
#define COUPLE_MODE_PEER 2u
#define COUPLE_MODE_P2P 3u

/* ---- COUPLE_MODE flags (optional second payload byte) ----
 * PREDICT: in PEER mode, couple to where the peer shaft is NOW --
 * peer_pos + peer_vel * (its age + wire time) -- instead of to its last
 * received position. A spring on a delayed position behaves like a spring
 * plus NEGATIVE damping that grows with Kp; that is what makes the pair
 * buzz hands-off at stiffnesses one motor holds quietly. The age is measured
 * per frame, so this also tracks the 0-1 ms drift between the two ESCs'
 * unsynchronised 1 kHz ticks. */
#define COUPLE_FLAG_PREDICT (1u << 0)
#define COUPLE_PREDICT_WIRE_US 130u     /* 8-byte frame at 1 Mbit/s, before
                                           the Rx stamp; not in the age */
#define COUPLE_PREDICT_MAX_US 3000u     /* never extrapolate further */

/* ---- Defaults, used from boot until the Pi sends COUPLE_GAINS/LOCAL ----
 * Starting point: the gains position_mirror_test.py runs stably on the Pi.
 * Once bench tuning settles, bake the results in here so the pair couples
 * on sensible values without being told. Every one can also be overridden
 * per build with -D. */
#ifndef COUPLE_DEFAULT_KP
#define COUPLE_DEFAULT_KP 0.000265f /* A/count */
#endif
#ifndef COUPLE_DEFAULT_KD
#define COUPLE_DEFAULT_KD 0.00001f /* A/(count/s), on own_vel - peer_vel */
#endif
#ifndef COUPLE_DEFAULT_KD_LOCAL
#define COUPLE_DEFAULT_KD_LOCAL 0.0f /* A/(count/s), on own_vel only */
#endif
#ifndef COUPLE_DEFAULT_IQ_MAX_A
#define COUPLE_DEFAULT_IQ_MAX_A 0.8f
#endif

/* Hard ceiling on Iq_max, whatever the Pi asks for. NOMINAL_CURRENT_A
 * (pmsm_motor_parameters.h) is 0.8 A, and nothing on the Iq reference path
 * clips at it -- readback has been seen at 0.83-0.85 A -- so this does. */
#define COUPLE_IQ_MAX_CEILING_A NOMINAL_CURRENT_A

/* sign_check_test.py: positive Iq increases the encoder count on both
 * motors, matching SIGN = 1 in the Pi scripts. Flip per build if a motor is
 * ever rewired. A wrong sign is positive feedback: the shafts run away
 * until COUPLE_ERR_LIMIT_CNT or COUPLE_VEL_LIMIT_CNT_S trips. */
#ifndef COUPLE_TORQUE_SIGN
#define COUPLE_TORQUE_SIGN 1.0f
#endif

/* EMA on each shaft's velocity: vel = a*raw + (1-a)*vel. At 1 kHz and 4000
 * counts/rev one count per tick is 1000 counts/s, so raw velocity is
 * coarse; 0.5 matches VEL_FILTER_ALPHA in position_mirror_test.py. Lower
 * it to smooth a stiff coupling's D term, at the price of phase lag. */
#define COUPLE_VEL_FILTER_ALPHA 0.5f

/* Peer TELEM older than this disengages PEER mode (trip PEER_TIMEOUT) and
 * blocks engagement. The peer sends at 1 kHz, so this is 5 missed frames. */
#define COUPLE_PEER_TIMEOUT_US 5000u

/* A requested mode engages only once both shafts are slower than this, so
 * both ESCs latch their offsets from the same physical state. Engaging
 * while one shaft is moving would latch slightly different offsets on the
 * two sides, and they would then fight each other with a constant preload
 * of Kp * (the mismatch). Hands off at engage and it is immediate. */
#define COUPLE_ENGAGE_VEL_MAX_CNT_S 2000.0f

/* Runaway limits while engaged (same values as position_mirror_test.py's
 * watchdog). Tripping disengages, zeroes Iq and latches the trip code. */
#define COUPLE_ERR_LIMIT_CNT 40000          /* 10 revolutions */
#define COUPLE_VEL_LIMIT_CNT_S 200000.0f    /* 50 rev/s, own shaft */

/* Direct mode: a SET_IQ not refreshed within this zeroes the current (trip
 * HOST_TIMEOUT); the next SET_IQ resumes. Closes the gap where a crashed Pi
 * script left the motor on its last command forever. Scripts that send one
 * SET_IQ and then sleep (spin_motor2.py, sign_check_test.py) must re-send
 * it at least this often against this firmware. 0 disables. */
#ifndef COUPLE_DIRECT_TIMEOUT_US
#define COUPLE_DIRECT_TIMEOUT_US 20000u /* was 200000u; see FIRMWARE_CHANGES.txt */
#endif

/* ---- P2P mode (COUPLE_MODE_P2P) ----
 *
 * Symmetric bilateral control after Ohnishi/Katsura (Keio): no leader, no
 * follower, the same law on both ESCs with own/peer swapped. Each motor is
 * made into a nominal inertia by a disturbance observer (DOB); a reaction
 * force observer (RFOB) estimates the external force on each shaft; and
 * the control is split into two decoupled modes:
 *
 *   differential  x_own - x_peer -> 0     position servo, acceleration units
 *   common        f_own + f_peer -> 0     force servo (action = reaction)
 *
 * Every 1 kHz tick, per shaft k in {own, peer} (the peer's from its TELEM:
 * encoder and 1 ms mean Iq, so this ESC runs BOTH observers itself and no
 * new frame is needed):
 *
 *   DOB    z_k += min(1, g*dt) * (i_k + (g/G_k)*v_k - z_k)
 *          d_k  = z_k - (g/G_k)*v_k          disturbance, amps
 *   RFOB   f_k  = -(d_k - comp*Fc_k*sat(v_k/V0))   external force, amps,
 *                                                  + = toward +counts
 *   ddx    = 0.5*( Kf*(f_own + f_peer) - Kp*err - Kv*derr )   counts/s^2
 *   iq     = clamp( ddx/G_own + d_own, +-Iq_max )
 *
 * i_k is MEASURED current (TELEM's 1 ms mean), not the command, so the
 * observer stays right when the output clamps -- no wind-up. err and derr
 * are the PEER-mode error terms (predict flag included).
 *
 * What the gains mean:
 *   Kp, Kv  = wn^2, 2*zeta*wn: the differential mode's natural frequency and
 *           damping. Because the DOB cancels friction and load, steady-state
 *           position error is driven to zero (the DOB acts as an integrator)
 *           -- that is what removes PEER mode's spring.
 *   Kf      sets the inertia you feel: a push f on one shaft accelerates
 *           both at Kf*f/2, so the apparent mass is 2/Kf. 100 % = 2*G_B,
 *           i.e. the pair feels like motor B alone (PEER mode: both rotors).
 *   g       observer cutoff. Higher = stiffer, noisier (it differentiates a
 *           4000-count encoder at 1 kHz).
 *   comp    how much of the nominal Coulomb friction the RFOB subtracts.
 *           0 = you feel both shafts' friction (honest physics); 100 % =
 *           compensated, lighter, but it chatters at rest on 1-count
 *           velocity noise (p2p_sim.py).
 *
 * Defaults from p2p_sim.py (2026-11-18): wn 20 Hz, zeta 1.0, g 150 rad/s,
 * Kf 100 %, comp 0. The model goes unstable at Kv above ~500 /s (the 1 ms
 * peer delay); the defaults put Kv at 251 /s. Each is a SET_PARAM
 * (can_driver.h), applied immediately, engaged or not. */
#ifndef COUPLE_P2P_WN_DHZ_DEFAULT
#define COUPLE_P2P_WN_DHZ_DEFAULT 200 /* 0.1 Hz units: 20.0 Hz */
#endif
#ifndef COUPLE_P2P_ZETA_PCT_DEFAULT
#define COUPLE_P2P_ZETA_PCT_DEFAULT 100
#endif
#ifndef COUPLE_P2P_KF_PCT_DEFAULT
#define COUPLE_P2P_KF_PCT_DEFAULT 100
#endif
#ifndef COUPLE_P2P_DOB_G_DEFAULT
#define COUPLE_P2P_DOB_G_DEFAULT 150 /* rad/s */
#endif
#ifndef COUPLE_P2P_FRIC_PCT_DEFAULT
#define COUPLE_P2P_FRIC_PCT_DEFAULT 0
#endif

/* SET_PARAM ranges (outside = rejected and counted, like Tafter). */
#define COUPLE_P2P_WN_DHZ_MIN 10    /* 1 Hz */
#define COUPLE_P2P_WN_DHZ_MAX 400   /* 40 Hz */
#define COUPLE_P2P_ZETA_PCT_MIN 20
#define COUPLE_P2P_ZETA_PCT_MAX 300
#define COUPLE_P2P_KF_PCT_MIN 0     /* 0 = no force channel: the pair locks */
#define COUPLE_P2P_KF_PCT_MAX 300
#define COUPLE_P2P_DOB_G_MIN 20
#define COUPLE_P2P_DOB_G_MAX 1000
#define COUPLE_P2P_FRIC_PCT_MIN 0
#define COUPLE_P2P_FRIC_PCT_MAX 150

/* Plant per node, indexed by CAN_NODE_ID (0 = ESC 1 = motor A, with the 3D
 * arm; 1 = ESC 2 = motor B). Both boards carry both rows: each runs the
 * other's observer too. G from TEST_LOG 2026-11-15 (A2 re-run on the 3D
 * arm); Coulomb from follower_model_arm3d.json and leader_comp_B.json. */
#define COUPLE_P2P_G_NODE0 4.082e6f  /* counts/s^2 per A */
#define COUPLE_P2P_G_NODE1 8.877e6f
#define COUPLE_P2P_FC_NODE0 0.025f   /* A */
#define COUPLE_P2P_FC_NODE1 0.017f
#define COUPLE_P2P_FRIC_V0_CNT_S 300.0f /* sat() knee of the Coulomb model */

/* ---- Ripple (cogging) compensation, CHANGE 10 ----
 *
 * Each motor has an angle-locked torque ripple (12-slot cogging at 12/24/
 * 36/rev, current-offset 7/rev, 14/28/42/84/rev) plus, on arm A, gravity
 * at 1/rev. Without a model the RFOB reports it as EXTERNAL FORCE, so the
 * force servo hands both motors' ripple to the operator. A fixed map cannot
 * work (TEST_LOG 2026-10-26: the 12/rev phase moves per power-up, the 7/rev
 * amplitude per START), so the Pi measures it in-session (p2p_cal.py) and
 * sends both motors' tables to both ESCs:
 *
 *   r_n(theta) = sum_h a_h cos(h theta) + b_h sin(h theta),
 *   theta = 2 pi (raw TIM4 count) / 4000, h in COUPLE_RIPPLE_HARMONICS,
 *   in amps to OVERCOME it (motor current frame, as leader_cal.py fits).
 *
 * With ripple_pct = p (0..150 %, 0 = off):
 *   observer input   i_k -> i_k - p*r_k(theta_k)  (own AND peer), so the
 *                    RFOB no longer sees ripple as force;
 *   own output       iq += p*r_own(theta_own)     feed-forward, so the
 *                    motor overcomes its own ripple without observer lag.
 * The table's Coulomb term replaces the nominal one in the friction comp.
 *
 * SET_PARAM ids (int32 microamps; |coef| <= 0.3 A, Coulomb 0..0.2 A):
 *   32 + 2k / 33 + 2k  node 0, harmonic k: cos / sin coefficient
 *   50                 node 0 Coulomb
 *   64 + 2k / 65 + 2k, 82   node 1, the same */
#define COUPLE_RIPPLE_N_HARM 9
#define COUPLE_RIPPLE_HARMONICS {1, 7, 12, 14, 24, 28, 36, 42, 84}
#define COUPLE_RIPPLE_COEF_MAX_UA 300000
#define COUPLE_RIPPLE_FC_MAX_UA 200000
#define COUPLE_P2P_RIPPLE_PCT_MIN 0
#define COUPLE_P2P_RIPPLE_PCT_MAX 150

/* Kf at 100 %: 2*G_B, the same on both boards (symmetry needs that). */
#define COUPLE_P2P_KF_REF (2.0f * COUPLE_P2P_G_NODE1)

/* A measured Iq beyond this is not a current this firmware can command
 * (clamp 0.8 A, readback seen to 0.85 A); it is TELEM from a board that is
 * not in RUN (IDLE reads +-25 A with no offset calibration) or garbage.
 * Blocks engagement; trips OBS_RANGE while engaged. */
#define COUPLE_P2P_IQ_SANE_A 1.2f

/* ---- DBG status byte ---- */
#define COUPLE_STATUS_RUN (1u << 0)       /* motor state machine is in RUN */
#define COUPLE_STATUS_ENGAGED (1u << 1)   /* coupling law is driving Iq */
#define COUPLE_STATUS_REQUESTED (1u << 2) /* a mode is requested (maybe
                                             still waiting to engage) */
#define COUPLE_STATUS_MODE_PEER (1u << 3) /* that mode uses the peer:
                                             PEER or P2P (not HOLD). The Pi
                                             knows which one it asked for. */
#define COUPLE_STATUS_SATURATED (1u << 4) /* Iq clamped at Iq_max */
#define COUPLE_STATUS_TRIP_SHIFT 5u       /* bits 5..7: last trip code */
#define COUPLE_STATUS_TRIP_MASK (7u << COUPLE_STATUS_TRIP_SHIFT)

/* Trip codes. Latched until the next COUPLE_MODE request (or, for
 * HOST_TIMEOUT, the next SET_IQ). */
#define COUPLE_TRIP_NONE 0u
#define COUPLE_TRIP_PEER_TIMEOUT 1u /* peer TELEM went stale while engaged */
#define COUPLE_TRIP_ERR_LIMIT 2u    /* |err| > COUPLE_ERR_LIMIT_CNT */
#define COUPLE_TRIP_VEL_LIMIT 3u    /* |own_vel| > COUPLE_VEL_LIMIT_CNT_S */
#define COUPLE_TRIP_HOST_TIMEOUT 4u /* direct SET_IQ went stale */
#define COUPLE_TRIP_LEFT_RUN 5u     /* motor left RUN while engaged (fault) */
#define COUPLE_TRIP_OBS_RANGE 6u    /* P2P: a measured Iq the observers read
                                       was out of range (peer not in RUN?) */

typedef struct {
  float iq_cmd_a;       /* Iq commanded this tick; 0 when not in RUN */
  int32_t err_counts;   /* coupling error; 0 when not engaged */
  uint8_t flags;        /* COUPLE_STATUS_* */
  uint32_t peer_age_us; /* UINT32_MAX until the first peer frame */
} Couple_Status_t;

/* All of these run in the 1 kHz hook context: Couple_Tick() from
 * CAN_ControlTick(), the setters from CAN_ProcessPendingMessages() just
 * before it. Nothing here is safe to call from an interrupt. */
void Couple_Tick(const CAN_TickSample_t* s);

bool Couple_SetGains(float kp, float kd);         /* false = rejected */
bool Couple_SetLocal(float kd_local, float iq_max);
void Couple_RequestMode(uint8_t mode, uint8_t flags); /* COUPLE_FLAG_* */
void Couple_SetDirectIq(float amps);

/* P2P tuning, from SET_PARAM ids CAN_PARAM_P2P_* (can_driver.h). Returns
 * false for an unknown id or an out-of-range value (the caller counts it). */
bool Couple_SetP2PParam(uint8_t id, int32_t value);

void Couple_GetStatus(Couple_Status_t* out);

#endif /* COUPLE_CTRL_H */

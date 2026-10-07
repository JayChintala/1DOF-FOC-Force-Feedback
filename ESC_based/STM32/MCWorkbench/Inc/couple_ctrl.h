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
#define COUPLE_DIRECT_TIMEOUT_US 200000u
#endif

/* ---- DBG status byte ---- */
#define COUPLE_STATUS_RUN (1u << 0)       /* motor state machine is in RUN */
#define COUPLE_STATUS_ENGAGED (1u << 1)   /* coupling law is driving Iq */
#define COUPLE_STATUS_REQUESTED (1u << 2) /* a mode is requested (maybe
                                             still waiting to engage) */
#define COUPLE_STATUS_MODE_PEER (1u << 3) /* that mode is PEER, not HOLD */
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

void Couple_GetStatus(Couple_Status_t* out);

#endif /* COUPLE_CTRL_H */

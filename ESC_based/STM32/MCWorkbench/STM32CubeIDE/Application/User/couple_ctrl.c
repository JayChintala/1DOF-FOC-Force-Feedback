/*
 * couple_ctrl.c
 *
 * See couple_ctrl.h for the modes and the law.
 *
 * Everything here runs in the 1 kHz hook context (SysTick, NVIC priority 4),
 * one tick at a time, so the state below needs no locking. The only other
 * parties are the FDCAN Rx interrupt (priority 5, which this context
 * outranks -- see CAN_GetPeerSample()) and the 16 kHz FOC interrupt, which
 * only consumes the Iq reference set here.
 */

#include "couple_ctrl.h"

#include <math.h>
#include <string.h>

#include "mc_api.h"
#include "mc_config.h"             /* pMCI[] */
#include "parameters_conversion.h" /* M1_PULSE_NBR, NOMINAL_CURRENT_A */

/* TIM4 counts 0..M1_PULSE_NBR: 4000 states per revolution. */
#define ENC_MODULUS ((int32_t)M1_PULSE_NBR + 1)
#define ENC_HALF (ENC_MODULUS / 2)

typedef struct {
  /* Tuning: couple_ctrl.h defaults until the Pi sends its own. */
  float kp;
  float kd;
  float kd_local;
  float iq_max;

  /* This shaft, unwrapped. */
  bool own_valid;
  uint16_t own_raw;
  int32_t own_pos;
  uint32_t own_us;
  float own_vel;

  /* The peer's shaft, unwrapped, from its TELEM frames. */
  bool peer_valid;
  uint32_t peer_seq;
  uint16_t peer_raw;
  int32_t peer_pos;
  uint16_t peer_ts;
  uint32_t peer_arrival_us;
  float peer_vel;
  uint32_t peer_age_us;

  /* req_mode is what was asked for; engaged says whether the law has
     latched its offsets and is actually driving Iq. */
  uint8_t req_mode;
  bool engaged;
  int32_t own0;
  int32_t peer0;

  /* Direct (SET_IQ) mode. */
  bool direct_pending;
  float direct_value;
  bool direct_armed;
  uint32_t direct_us;

  /* Reported in DBG. */
  bool run;
  float iq_cmd;
  int32_t err;
  bool saturated;
  uint8_t trip;
} Couple_State_t;

static Couple_State_t s_couple = {
    .kp = COUPLE_DEFAULT_KP,
    .kd = COUPLE_DEFAULT_KD,
    .kd_local = COUPLE_DEFAULT_KD_LOCAL,
    .iq_max = COUPLE_DEFAULT_IQ_MAX_A,
    .peer_age_us = UINT32_MAX,
    .req_mode = COUPLE_MODE_OFF,
};

/* The build uses -Ofast, which implies -ffinite-math-only: GCC may fold
 * isfinite() to true and assume NaN never reaches a comparison. Values off
 * the bus can be anything, so test the exponent bits directly. */
static bool IsFinite(float x) {
  uint32_t bits;
  memcpy(&bits, &x, sizeof(bits));
  return (bits & 0x7F800000u) != 0x7F800000u;
}

/* Advances an unwrapped position by the shortest step from the previous raw
 * count. Valid while the shaft moves under half a revolution between
 * samples: 2000 counts per millisecond, i.e. 30000 rpm. */
static int32_t Unwrap(int32_t pos, uint16_t* last_raw, uint16_t raw) {
  int32_t delta = (int32_t)raw - (int32_t)*last_raw;
  if (delta > ENC_HALF) {
    delta -= ENC_MODULUS;
  } else if (delta < -ENC_HALF) {
    delta += ENC_MODULUS;
  }
  *last_raw = raw;
  return pos + delta;
}

static float FilterVel(float prev, float raw) {
  return COUPLE_VEL_FILTER_ALPHA * raw +
         (1.0f - COUPLE_VEL_FILTER_ALPHA) * prev;
}

/* Sets the Iq reference and applies it NOW. MC_SetCurrentReferenceMotor1_F()
 * only buffers the command: the MF task executes buffered commands at its
 * start, and this runs at its end (PostMediumFrequencyHook), so left alone
 * every reference would wait a full millisecond before the current loop
 * saw it -- an extra tick of delay inside the coupling loop, which is the
 * one thing this firmware exists to remove. MCI_ExecBufferedCommands() is
 * exactly what the MF task would call, from the same context, one tick
 * early. In anything but RUN the command is dropped, as it always was. */
static void CommandIq(float amps) {
  if (MC_GetSTMStateMotor1() != RUN) {
    return;
  }
  qd_f_t iqdRef;
  iqdRef.q = amps;
  iqdRef.d = 0.0f;
  MC_SetCurrentReferenceMotor1_F(iqdRef);
  MCI_ExecBufferedCommands(pMCI[M1]);
  s_couple.iq_cmd = amps;
}

/* Drops any coupling mode and zeroes Iq. A trip code latches a reason for
 * the Pi; COUPLE_TRIP_NONE (a requested OFF) keeps whatever was latched. */
static void Disengage(uint8_t trip) {
  s_couple.engaged = false;
  s_couple.req_mode = COUPLE_MODE_OFF;
  s_couple.err = 0;
  s_couple.saturated = false;
  s_couple.direct_armed = false;
  s_couple.direct_pending = false;
  if (trip != COUPLE_TRIP_NONE) {
    s_couple.trip = trip;
  }
  CommandIq(0.0f);
}

static void UpdateOwn(const CAN_TickSample_t* s) {
  if (!s_couple.own_valid) {
    s_couple.own_raw = s->enc;
    s_couple.own_pos = (int32_t)s->enc;
    s_couple.own_us = s->us;
    s_couple.own_vel = 0.0f;
    s_couple.own_valid = true;
    return;
  }

  int32_t const pos = Unwrap(s_couple.own_pos, &s_couple.own_raw, s->enc);
  uint32_t const dt_us = s->us - s_couple.own_us;
  if (dt_us > 0u) {
    float const raw = (float)(pos - s_couple.own_pos) * 1.0e6f / (float)dt_us;
    s_couple.own_vel = FilterVel(s_couple.own_vel, raw);
  }
  s_couple.own_pos = pos;
  s_couple.own_us = s->us;
}

static void UpdatePeer(const CAN_TickSample_t* s) {
  CAN_PeerSample_t p;
  if (CAN_GetPeerSample(s, &p) && (p.seq != s_couple.peer_seq)) {
    /* After a gap longer than the timeout, the peer's uint16 timestamp may
     * have wrapped and its shaft may have moved any distance, so velocity
     * restarts from this sample instead of being differenced across the
     * gap. Position keeps unwrapping best-effort; nothing relies on it
     * being continuous across a gap that long, because the gap has already
     * tripped PEER mode and re-engaging re-latches peer0. */
    bool const gap = !s_couple.peer_valid ||
                     ((s->us - s_couple.peer_arrival_us) >
                      COUPLE_PEER_TIMEOUT_US);

    if (!s_couple.peer_valid) {
      s_couple.peer_raw = p.enc;
      s_couple.peer_pos = (int32_t)p.enc;
      s_couple.peer_valid = true;
    } else {
      int32_t const pos = Unwrap(s_couple.peer_pos, &s_couple.peer_raw, p.enc);
      /* dt from the PEER's clock, stamped when it sampled its encoder:
         immune to bus arbitration and to when this tick happened to run. */
      uint16_t const dt_us = (uint16_t)(p.ts_us - s_couple.peer_ts);
      if (!gap && (dt_us > 0u)) {
        float const raw =
            (float)(pos - s_couple.peer_pos) * 1.0e6f / (float)dt_us;
        s_couple.peer_vel = FilterVel(s_couple.peer_vel, raw);
      }
      s_couple.peer_pos = pos;
    }
    if (gap) {
      s_couple.peer_vel = 0.0f;
    }
    s_couple.peer_ts = p.ts_us;
    s_couple.peer_arrival_us = p.arrival_us;
    s_couple.peer_seq = p.seq;
  }

  s_couple.peer_age_us =
      s_couple.peer_valid ? (s->us - s_couple.peer_arrival_us) : UINT32_MAX;
}

static void RunLaw(bool peer_fresh) {
  bool const peer = (s_couple.req_mode == COUPLE_MODE_PEER);
  if (peer && !peer_fresh) {
    Disengage(COUPLE_TRIP_PEER_TIMEOUT);
    return;
  }

  int32_t err = s_couple.own_pos - s_couple.own0;
  float derr = s_couple.own_vel;
  if (peer) {
    err -= s_couple.peer_pos - s_couple.peer0;
    derr -= s_couple.peer_vel;
  }

  if ((err > COUPLE_ERR_LIMIT_CNT) || (err < -COUPLE_ERR_LIMIT_CNT)) {
    Disengage(COUPLE_TRIP_ERR_LIMIT);
    return;
  }
  if (fabsf(s_couple.own_vel) > COUPLE_VEL_LIMIT_CNT_S) {
    Disengage(COUPLE_TRIP_VEL_LIMIT);
    return;
  }

  float const iq_max = fminf(s_couple.iq_max, COUPLE_IQ_MAX_CEILING_A);
  float const raw = -COUPLE_TORQUE_SIGN *
                    (s_couple.kp * (float)err + s_couple.kd * derr +
                     s_couple.kd_local * s_couple.own_vel);
  float iq = raw;
  if (iq > iq_max) {
    iq = iq_max;
  } else if (iq < -iq_max) {
    iq = -iq_max;
  }
  s_couple.saturated = (iq != raw);
  s_couple.err = err;
  CommandIq(iq);
}

static void RunDirect(const CAN_TickSample_t* s) {
  if (s_couple.direct_pending) {
    s_couple.direct_pending = false;
    CommandIq(s_couple.direct_value);
    s_couple.direct_armed = true;
    s_couple.direct_us = s->us;
    if (s_couple.trip == COUPLE_TRIP_HOST_TIMEOUT) {
      s_couple.trip = COUPLE_TRIP_NONE;
    }
    return;
  }
#if COUPLE_DIRECT_TIMEOUT_US > 0
  if (s_couple.direct_armed &&
      ((s->us - s_couple.direct_us) > COUPLE_DIRECT_TIMEOUT_US)) {
    s_couple.direct_armed = false;
    s_couple.trip = COUPLE_TRIP_HOST_TIMEOUT;
    CommandIq(0.0f);
  }
#endif
}

void Couple_Tick(const CAN_TickSample_t* s) {
  UpdateOwn(s);
  UpdatePeer(s);

  bool const peer_fresh = s_couple.peer_age_us <= COUPLE_PEER_TIMEOUT_US;
  s_couple.run = (MC_GetSTMStateMotor1() == RUN);

  if (!s_couple.run) {
    /* A clean STOP has already cancelled the mode through
       Couple_RequestMode(), so arriving here engaged means the motor
       faulted out of RUN. A request that never engaged is kept, and
       engages once the motor is back in RUN. */
    if (s_couple.engaged) {
      Disengage(COUPLE_TRIP_LEFT_RUN);
    }
    s_couple.direct_armed = false;
    s_couple.direct_pending = false; /* SET_IQ outside RUN is dropped */
    s_couple.iq_cmd = 0.0f;
    return;
  }

  if (!s_couple.engaged && (s_couple.req_mode != COUPLE_MODE_OFF)) {
    bool settled = fabsf(s_couple.own_vel) <= COUPLE_ENGAGE_VEL_MAX_CNT_S;
    if (s_couple.req_mode == COUPLE_MODE_PEER) {
      settled = settled && peer_fresh &&
                (fabsf(s_couple.peer_vel) <= COUPLE_ENGAGE_VEL_MAX_CNT_S);
    }
    if (settled) {
      s_couple.own0 = s_couple.own_pos;
      s_couple.peer0 = s_couple.peer_pos;
      s_couple.engaged = true;
    }
  }

  if (s_couple.engaged) {
    RunLaw(peer_fresh);
  } else if (s_couple.req_mode == COUPLE_MODE_OFF) {
    RunDirect(s);
  } else {
    /* Requested but not settled yet: Iq stays at the zero the request set,
       and SET_IQ is ignored rather than fighting the pending mode. */
    s_couple.direct_pending = false;
  }
}

bool Couple_SetGains(float kp, float kd) {
  if (!IsFinite(kp) || !IsFinite(kd) || (kp < 0.0f) || (kd < 0.0f)) {
    return false; /* a negative gain is positive feedback */
  }
  s_couple.kp = kp;
  s_couple.kd = kd;
  return true;
}

bool Couple_SetLocal(float kd_local, float iq_max) {
  if (!IsFinite(kd_local) || !IsFinite(iq_max) || (kd_local < 0.0f) ||
      (iq_max <= 0.0f)) {
    return false;
  }
  s_couple.kd_local = kd_local;
  s_couple.iq_max = fminf(iq_max, COUPLE_IQ_MAX_CEILING_A);
  return true;
}

void Couple_RequestMode(uint8_t mode) {
  if (mode == COUPLE_MODE_OFF) {
    if (s_couple.engaged || (s_couple.req_mode != COUPLE_MODE_OFF)) {
      Disengage(COUPLE_TRIP_NONE);
    }
    return;
  }
  if ((mode != COUPLE_MODE_HOLD) && (mode != COUPLE_MODE_PEER)) {
    return;
  }

  /* Every request -- a repeat of the current mode included -- starts over:
     Iq to zero, trip cleared, offsets re-latched on the next settled tick.
     Sending the mode again is how you re-zero a coupling that slipped. */
  Disengage(COUPLE_TRIP_NONE);
  s_couple.trip = COUPLE_TRIP_NONE;
  s_couple.req_mode = mode;
}

void Couple_SetDirectIq(float amps) {
  if (!IsFinite(amps)) {
    return;
  }
  s_couple.direct_value = amps;
  s_couple.direct_pending = true; /* applied by this tick's Couple_Tick() */
}

void Couple_GetStatus(Couple_Status_t* out) {
  uint8_t flags = (uint8_t)((s_couple.trip << COUPLE_STATUS_TRIP_SHIFT) &
                            COUPLE_STATUS_TRIP_MASK);
  if (s_couple.run) {
    flags |= (uint8_t)COUPLE_STATUS_RUN;
  }
  if (s_couple.engaged) {
    flags |= (uint8_t)COUPLE_STATUS_ENGAGED;
  }
  if (s_couple.req_mode != COUPLE_MODE_OFF) {
    flags |= (uint8_t)COUPLE_STATUS_REQUESTED;
  }
  if (s_couple.req_mode == COUPLE_MODE_PEER) {
    flags |= (uint8_t)COUPLE_STATUS_MODE_PEER;
  }
  if (s_couple.saturated) {
    flags |= (uint8_t)COUPLE_STATUS_SATURATED;
  }

  out->iq_cmd_a = s_couple.iq_cmd;
  out->err_counts = s_couple.err;
  out->flags = flags;
  out->peer_age_us = s_couple.peer_age_us;
}

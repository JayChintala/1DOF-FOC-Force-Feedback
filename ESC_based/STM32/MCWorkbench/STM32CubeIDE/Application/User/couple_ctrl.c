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

#if (CAN_NODE_ID > 1) || (CAN_PEER_NODE_ID > 1)
#error "P2P plant table (COUPLE_P2P_G_NODE*) only covers nodes 0 and 1"
#endif
#define P2P_G_OF(node) ((node) == 0 ? COUPLE_P2P_G_NODE0 : COUPLE_P2P_G_NODE1)
#define P2P_FC_OF(node) ((node) == 0 ? COUPLE_P2P_FC_NODE0 : COUPLE_P2P_FC_NODE1)
#define P2P_TS_S 1.0e-3f /* own tick; the peer's dt comes from its stamps */
#define TWO_PI_F 6.28318531f

/* Ripple tables (CHANGE 10), per node: [2k] cos, [2k+1] sin, amps. */
typedef struct {
  float ab[2 * COUPLE_RIPPLE_N_HARM];
  float fc; /* Coulomb, amps; the nominal value until a table arrives */
} Ripple_t;

static const uint16_t kRippleHarm[COUPLE_RIPPLE_N_HARM] =
    COUPLE_RIPPLE_HARMONICS;
static Ripple_t s_ripple[2] = {
    {.fc = COUPLE_P2P_FC_NODE0},
    {.fc = COUPLE_P2P_FC_NODE1},
};

/* Quarter-wave sine table over the 4000-count circle: s_sinQ[k] =
   sin(2 pi k / 4000), k = 0..1000. Built once (SinTableInit()). NOT the
   MCSDK's MCM_Trig_Functions(): that drives the hardware CORDIC, which the
   16 kHz FOC interrupt also uses and can preempt mid-operation. */
#define SIN_Q (ENC_MODULUS / 4)
static float s_sinQ[SIN_Q + 1];
static bool s_sinReady = false;

static void SinTableInit(void) {
  for (int32_t k = 0; k <= SIN_Q; k++) {
    s_sinQ[k] = sinf(6.28318531f * (float)k / (float)ENC_MODULUS);
  }
  s_sinReady = true;
}

/* sin(2 pi idx / 4000) for any idx >= 0. */
static float SinIdx(uint32_t idx) {
  uint32_t const k = idx % (uint32_t)ENC_MODULUS;
  uint32_t const q = k / (uint32_t)SIN_Q;
  uint32_t const r = k % (uint32_t)SIN_Q;
  switch (q) {
    case 0u:
      return s_sinQ[r];
    case 1u:
      return s_sinQ[SIN_Q - r];
    case 2u:
      return -s_sinQ[r];
    default:
      return -s_sinQ[SIN_Q - r];
  }
}

/* r_node(theta) in amps (motor current frame), unscaled by ripple_pct. */
static float RippleAt(uint32_t node, uint16_t enc) {
  Ripple_t const* t = &s_ripple[node];
  float r = 0.0f;
  for (uint32_t k = 0u; k < (uint32_t)COUPLE_RIPPLE_N_HARM; k++) {
    uint32_t const idx = (uint32_t)kRippleHarm[k] * (uint32_t)enc;
    r += t->ab[2u * k] * SinIdx(idx + (uint32_t)SIN_Q) /* cos */
         + t->ab[2u * k + 1u] * SinIdx(idx);
  }
  return r;
}

/* Raw count halfway between two consecutive raw counts, across the wrap. */
static uint16_t MidCount(uint16_t a0, uint16_t a1) {
  uint32_t sum = (uint32_t)a0 + (uint32_t)a1;
  if (((int32_t)a0 - (int32_t)a1 > ENC_HALF) ||
      ((int32_t)a1 - (int32_t)a0 > ENC_HALF)) {
    sum += (uint32_t)ENC_MODULUS;
  }
  return (uint16_t)((sum / 2u) % (uint32_t)ENC_MODULUS);
}

/* One disturbance observer (couple_ctrl.h, "P2P mode"): z is the low-passed
   (i + k*v), k = g/G, and d = z - k*v is the disturbance in amps. */
typedef struct {
  float z;
  float d;
  bool valid;
} Dob_t;

typedef struct {
  /* Tuning: couple_ctrl.h defaults until the Pi sends its own. */
  float kp;
  float kd;
  float kd_local;
  float iq_max;

  /* This shaft, unwrapped. */
  bool own_valid;
  uint16_t own_raw;
  uint16_t own_raw_prev; /* last tick's raw count (ripple midpoint) */
  int32_t own_pos;
  uint32_t own_us;
  float own_vel;

  /* The peer's shaft, unwrapped, from its TELEM frames. */
  bool peer_valid;
  uint32_t peer_seq;
  uint16_t peer_raw;
  uint16_t peer_raw_prev; /* ripple midpoint */
  int32_t peer_pos;
  uint16_t peer_ts;
  uint32_t peer_arrival_us;
  float peer_vel;
  uint32_t peer_age_us;

  /* P2P tuning (SET_PARAM units, as received) and what follows from it. */
  int32_t p2p_wn_dhz;
  int32_t p2p_zeta_pct;
  int32_t p2p_kf_pct;
  int32_t p2p_g;
  int32_t p2p_fric_pct;
  float p2p_kp;   /* wn^2, 1/s^2 */
  float p2p_kv;   /* 2*zeta*wn, 1/s */
  float p2p_kf;   /* counts/s^2 per A */
  float p2p_comp; /* 0..1.5 */
  int32_t p2p_ripple_pct;
  float p2p_ripple; /* 0..1.5 */

  /* P2P observers. Iq in the encoder frame (COUPLE_TORQUE_SIGN applied). */
  float own_iq;  /* this tick's 1 ms mean */
  float peer_iq; /* from the latest peer TELEM */
  float own_iq_eff;  /* minus the ripple table: what the observer sees */
  float peer_iq_eff;
  uint16_t own_enc; /* raw counts, for the ripple tables */
  Dob_t dob_own;
  Dob_t dob_peer;
  float f_own;  /* RFOB, last P2P tick: external force, amps */
  float f_peer;

  /* req_mode is what was asked for; engaged says whether the law has
     latched its offsets and is actually driving Iq. */
  uint8_t req_mode;
  bool predict; /* COUPLE_FLAG_PREDICT, latched with the mode request */
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
    .p2p_wn_dhz = COUPLE_P2P_WN_DHZ_DEFAULT,
    .p2p_zeta_pct = COUPLE_P2P_ZETA_PCT_DEFAULT,
    .p2p_kf_pct = COUPLE_P2P_KF_PCT_DEFAULT,
    .p2p_g = COUPLE_P2P_DOB_G_DEFAULT,
    .p2p_fric_pct = COUPLE_P2P_FRIC_PCT_DEFAULT,
    /* p2p_kp/kv/kf/comp are derived on the first tick (P2PDerive()). */
};
static bool s_p2pDerived = false;

/* Recomputes the P2P gains from the SET_PARAM values. */
static void P2PDerive(void) {
  float const wn = TWO_PI_F * 0.1f * (float)s_couple.p2p_wn_dhz;
  s_couple.p2p_kp = wn * wn;
  s_couple.p2p_kv = 2.0f * 0.01f * (float)s_couple.p2p_zeta_pct * wn;
  s_couple.p2p_kf = 0.01f * (float)s_couple.p2p_kf_pct * COUPLE_P2P_KF_REF;
  s_couple.p2p_comp = 0.01f * (float)s_couple.p2p_fric_pct;
  s_couple.p2p_ripple = 0.01f * (float)s_couple.p2p_ripple_pct;
  if (!s_sinReady) {
    SinTableInit();
  }
  s_p2pDerived = true;
}

/* Restarts an observer at rest-consistent state: d = i. */
static void DobReset(Dob_t* o, float iq, float vel, float g_plant) {
  float const k = (float)s_couple.p2p_g / g_plant;
  o->z = iq + k * vel;
  o->d = iq;
  o->valid = true;
}

/* One observer step over dt seconds. Forward Euler, as in p2p_sim.py; the
   gain is capped at 1 so a long gap cannot overshoot. */
static void DobStep(Dob_t* o, float iq, float vel, float g_plant, float dt) {
  float const g = (float)s_couple.p2p_g;
  float const k = g / g_plant;
  if (!o->valid) {
    DobReset(o, iq, vel, g_plant);
    return;
  }
  float a = g * dt;
  if (a > 1.0f) {
    a = 1.0f;
  }
  o->z += a * (iq + k * vel - o->z);
  o->d = o->z - k * vel;
}

static bool IqSane(float iq) {
  return (iq <= COUPLE_P2P_IQ_SANE_A) && (iq >= -COUPLE_P2P_IQ_SANE_A);
}

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

  /* P2P: this shaft's observer, on the current that drove the last 1 ms --
     the same interval the velocity above was differenced over. Run in every
     mode (it costs nothing) but only trusted from engagement, which resets
     it: before START the Iq readback is meaningless. */
  float const iq = IsFinite(s->iq_a) ? COUPLE_TORQUE_SIGN * s->iq_a : 1.0e9f;
  s_couple.own_iq = iq;
  s_couple.own_enc = s->enc;
  /* Ripple at the middle of the interval the velocity was differenced over:
     at 3000 counts/s that is 1.5 counts, 11 deg of 84/rev, vs 23 deg at
     either end. */
  uint16_t const mid = MidCount(s_couple.own_raw_prev, s->enc);
  s_couple.own_raw_prev = s->enc;
  s_couple.own_iq_eff =
      iq - COUPLE_TORQUE_SIGN * s_couple.p2p_ripple * RippleAt(CAN_NODE_ID, mid);
  if (IqSane(iq)) {
    DobStep(&s_couple.dob_own, s_couple.own_iq_eff, s_couple.own_vel,
            P2P_G_OF(CAN_NODE_ID),
            (dt_us > 0u) ? (float)dt_us * 1.0e-6f : P2P_TS_S);
  } else {
    s_couple.dob_own.valid = false;
  }
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

    /* P2P: the PEER's observer, run here on its own TELEM (1 ms mean Iq +
       encoder), stepped once per frame over the peer's own dt. A gap, or a
       reading no RUN-state board could produce, restarts it. */
    float const piq = IsFinite(p.iq_a) ? COUPLE_TORQUE_SIGN * p.iq_a : 1.0e9f;
    s_couple.peer_iq = piq;
    uint16_t const pmid = MidCount(s_couple.peer_raw_prev, p.enc);
    s_couple.peer_raw_prev = p.enc;
    s_couple.peer_iq_eff = piq - COUPLE_TORQUE_SIGN * s_couple.p2p_ripple *
                                     RippleAt(CAN_PEER_NODE_ID, pmid);
    if (gap || !IqSane(piq)) {
      s_couple.dob_peer.valid = false;
    } else {
      uint16_t const pdt_us = (uint16_t)(p.ts_us - s_couple.peer_ts);
      DobStep(&s_couple.dob_peer, s_couple.peer_iq_eff, s_couple.peer_vel,
              P2P_G_OF(CAN_PEER_NODE_ID),
              (pdt_us > 0u) ? (float)pdt_us * 1.0e-6f : P2P_TS_S);
    }
    s_couple.peer_ts = p.ts_us;
    s_couple.peer_arrival_us = p.arrival_us;
    s_couple.peer_seq = p.seq;
  }

  s_couple.peer_age_us =
      s_couple.peer_valid ? (s->us - s_couple.peer_arrival_us) : UINT32_MAX;
}

/* Nominal Coulomb friction at velocity vel, linear inside +-V0. */
static float P2PFriction(float fc, float vel) {
  float x = vel * (1.0f / COUPLE_P2P_FRIC_V0_CNT_S);
  if (x > 1.0f) {
    x = 1.0f;
  } else if (x < -1.0f) {
    x = -1.0f;
  }
  return fc * x;
}

static void RunLaw(bool peer_fresh) {
  bool const p2p = (s_couple.req_mode == COUPLE_MODE_P2P);
  bool const peer = (s_couple.req_mode == COUPLE_MODE_PEER) || p2p;
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
  float err_f = (float)err;
  if (peer && s_couple.predict) {
    /* peer_age_us is how long ago this frame left the Rx FIFO, on this
       tick; the wire time before that is fixed. Capped so a stale frame
       (which trips PEER_TIMEOUT anyway) is never extrapolated far. */
    uint32_t horizon_us = s_couple.peer_age_us + COUPLE_PREDICT_WIRE_US;
    if (horizon_us > COUPLE_PREDICT_MAX_US) {
      horizon_us = COUPLE_PREDICT_MAX_US;
    }
    err_f -= s_couple.peer_vel * ((float)horizon_us * 1.0e-6f);
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
  float raw;
  if (p2p) {
    /* Both observers must be live on sane readings: a peer that dropped
       out of RUN reads garbage Iq, and its "force" would be anything. */
    if (!s_couple.dob_own.valid || !s_couple.dob_peer.valid ||
        !IqSane(s_couple.peer_iq) || !IqSane(s_couple.own_iq)) {
      Disengage(COUPLE_TRIP_OBS_RANGE);
      return;
    }
    float const c = s_couple.p2p_comp;
    s_couple.f_own =
        -(s_couple.dob_own.d -
          c * P2PFriction(s_ripple[CAN_NODE_ID].fc, s_couple.own_vel));
    s_couple.f_peer =
        -(s_couple.dob_peer.d -
          c * P2PFriction(s_ripple[CAN_PEER_NODE_ID].fc, s_couple.peer_vel));
    float const ddx =
        0.5f * (s_couple.p2p_kf * (s_couple.f_own + s_couple.f_peer) -
                s_couple.p2p_kp * err_f - s_couple.p2p_kv * derr);
    raw = COUPLE_TORQUE_SIGN *
              (ddx / P2P_G_OF(CAN_NODE_ID) + s_couple.dob_own.d) +
          s_couple.p2p_ripple * RippleAt(CAN_NODE_ID, s_couple.own_enc);
  } else {
    raw = -COUPLE_TORQUE_SIGN *
          (s_couple.kp * err_f + s_couple.kd * derr +
           s_couple.kd_local * s_couple.own_vel);
  }
  float iq = raw;
  if (iq > iq_max) {
    iq = iq_max;
  } else if (iq < -iq_max) {
    iq = -iq_max;
  }
  s_couple.saturated = (iq != raw);
  s_couple.err = (int32_t)err_f; /* what the law used, predicted or not */
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
  if (!s_p2pDerived) {
    P2PDerive();
  }
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
    bool const p2p = (s_couple.req_mode == COUPLE_MODE_P2P);
    if ((s_couple.req_mode == COUPLE_MODE_PEER) || p2p) {
      settled = settled && peer_fresh &&
                (fabsf(s_couple.peer_vel) <= COUPLE_ENGAGE_VEL_MAX_CNT_S);
    }
    if (p2p) {
      /* Wait for a peer whose Iq readback is a RUN-state current. */
      settled = settled && IqSane(s_couple.peer_iq) && IqSane(s_couple.own_iq);
    }
    if (settled) {
      s_couple.own0 = s_couple.own_pos;
      s_couple.peer0 = s_couple.peer_pos;
      if (p2p) {
        /* Start both observers from this still state, so nothing read
           before RUN leaks into the first output. */
        DobReset(&s_couple.dob_own, s_couple.own_iq_eff, s_couple.own_vel,
                 P2P_G_OF(CAN_NODE_ID));
        DobReset(&s_couple.dob_peer, s_couple.peer_iq_eff, s_couple.peer_vel,
                 P2P_G_OF(CAN_PEER_NODE_ID));
        s_couple.f_own = 0.0f;
        s_couple.f_peer = 0.0f;
      }
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

void Couple_RequestMode(uint8_t mode, uint8_t flags) {
  if (mode == COUPLE_MODE_OFF) {
    if (s_couple.engaged || (s_couple.req_mode != COUPLE_MODE_OFF)) {
      Disengage(COUPLE_TRIP_NONE);
    }
    return;
  }
  if ((mode != COUPLE_MODE_HOLD) && (mode != COUPLE_MODE_PEER) &&
      (mode != COUPLE_MODE_P2P)) {
    return;
  }

  /* Every request -- a repeat of the current mode included -- starts over:
     Iq to zero, trip cleared, offsets re-latched on the next settled tick.
     Sending the mode again is how you re-zero a coupling that slipped. */
  Disengage(COUPLE_TRIP_NONE);
  s_couple.trip = COUPLE_TRIP_NONE;
  s_couple.req_mode = mode;
  s_couple.predict = (flags & COUPLE_FLAG_PREDICT) != 0u;
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
  if ((s_couple.req_mode == COUPLE_MODE_PEER) ||
      (s_couple.req_mode == COUPLE_MODE_P2P)) {
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

bool Couple_SetP2PParam(uint8_t id, int32_t value) {
  switch (id) {
    case CAN_PARAM_P2P_WN_DHZ:
      if ((value < COUPLE_P2P_WN_DHZ_MIN) || (value > COUPLE_P2P_WN_DHZ_MAX)) {
        return false;
      }
      s_couple.p2p_wn_dhz = value;
      break;
    case CAN_PARAM_P2P_ZETA_PCT:
      if ((value < COUPLE_P2P_ZETA_PCT_MIN) ||
          (value > COUPLE_P2P_ZETA_PCT_MAX)) {
        return false;
      }
      s_couple.p2p_zeta_pct = value;
      break;
    case CAN_PARAM_P2P_KF_PCT:
      if ((value < COUPLE_P2P_KF_PCT_MIN) || (value > COUPLE_P2P_KF_PCT_MAX)) {
        return false;
      }
      s_couple.p2p_kf_pct = value;
      break;
    case CAN_PARAM_P2P_DOB_G: {
      if ((value < COUPLE_P2P_DOB_G_MIN) || (value > COUPLE_P2P_DOB_G_MAX)) {
        return false;
      }
      /* d = z - k*v must not jump when k = g/G changes: move z with it. */
      float const dk = (float)(value - s_couple.p2p_g);
      s_couple.dob_own.z += dk / P2P_G_OF(CAN_NODE_ID) * s_couple.own_vel;
      s_couple.dob_peer.z +=
          dk / P2P_G_OF(CAN_PEER_NODE_ID) * s_couple.peer_vel;
      s_couple.p2p_g = value;
      break;
    }
    case CAN_PARAM_P2P_FRIC_PCT:
      if ((value < COUPLE_P2P_FRIC_PCT_MIN) ||
          (value > COUPLE_P2P_FRIC_PCT_MAX)) {
        return false;
      }
      s_couple.p2p_fric_pct = value;
      break;
    case CAN_PARAM_P2P_RIPPLE_PCT:
      if ((value < COUPLE_P2P_RIPPLE_PCT_MIN) ||
          (value > COUPLE_P2P_RIPPLE_PCT_MAX)) {
        return false;
      }
      s_couple.p2p_ripple_pct = value;
      break;
    default: {
      /* Ripple tables: 32..50 node 0, 64..82 node 1. */
      uint32_t node;
      if ((id >= CAN_PARAM_P2P_TABLE_NODE0) &&
          (id <= CAN_PARAM_P2P_TABLE_NODE0 + 2u * COUPLE_RIPPLE_N_HARM)) {
        node = 0u;
      } else if ((id >= CAN_PARAM_P2P_TABLE_NODE1) &&
                 (id <= CAN_PARAM_P2P_TABLE_NODE1 + 2u * COUPLE_RIPPLE_N_HARM)) {
        node = 1u;
      } else {
        return false;
      }
      uint32_t const k =
          (uint32_t)id - (node ? CAN_PARAM_P2P_TABLE_NODE1 : CAN_PARAM_P2P_TABLE_NODE0);
      if (k == 2u * COUPLE_RIPPLE_N_HARM) { /* Coulomb */
        if ((value < 0) || (value > COUPLE_RIPPLE_FC_MAX_UA)) {
          return false;
        }
        s_ripple[node].fc = 1.0e-6f * (float)value;
      } else {
        if ((value < -COUPLE_RIPPLE_COEF_MAX_UA) ||
            (value > COUPLE_RIPPLE_COEF_MAX_UA)) {
          return false;
        }
        s_ripple[node].ab[k] = 1.0e-6f * (float)value;
      }
      return true; /* nothing derived from the tables */
    }
  }
  P2PDerive();
  return true;
}

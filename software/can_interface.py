"""
CAN interface module for the 1DOF FOC Force-Feedback project.

Handles command TX (START / STOP / SET_IQ) and telemetry RX
(IQ_READBACK / ELEC_ANGLE / ENC_COUNT) over SocketCAN (can0, 1 Mbps).

Protocol (node base 0x000):
    0x001 START         Pi->MCU  no payload
    0x002 STOP          Pi->MCU  no payload
    0x003 SET_IQ        Pi->MCU  float32 LE, Amps
    0x010 IQ_READBACK    MCU->Pi float32 LE, Amps
    0x013 ELEC_ANGLE     MCU->Pi int16, DPP format
    0x014 ENC_COUNT      MCU->Pi uint32, raw TIM4 counter, wraps at M1_PULSE_NBR

Known gotcha: SET_IQ is silently dropped by firmware unless the motor is
already in RUN state. Always START, wait briefly, then SET_IQ.
"""

import struct
import threading
import time

import can

# ---- CAN arbitration IDs ----
ID_START = 0x001
ID_STOP = 0x002
ID_SET_IQ = 0x003
ID_IQ_READBACK = 0x010
ID_ELEC_ANGLE = 0x013
ID_ENC_COUNT = 0x014

# ENC_PULSE_NBR: wrap modulus of the raw TIM4 encoder counter.
# Confirmed from MCWorkbench\Src\mc_config_common.c:
#   .PulseNumber = M1_ENCODER_PPR * 4
# with M1_ENCODER_PPR = 1000 (pmsm_motor_parameters.h) -> 4000.
# NOTE: this is intentionally different from the M1_PULSE_NBR macro in
# parameters_conversion.h, which equals (4*PPR)-1 = 3999 -- that's the
# TIM4 ARR (auto-reload) value used to load the timer, so the counter
# counts 0..3999 (4000 distinct states) before wrapping. .PulseNumber
# (4000) is the value the encoder driver itself uses in its position
# math, so 4000 is the correct modulus here.
ENC_PULSE_NBR = 4000

DPP_TO_DEG = 180.0 / 32768.0
# ------------------------------------------------------------------------


class EncoderUnwrapper:
    """
    Converts a wrapping uint32 encoder counter into a continuous float
    count by tracking wraparounds between consecutive samples.

    Assumes consecutive samples never differ by more than half a
    revolution's worth of counts (i.e. you're sampling fast enough
    relative to shaft speed). If you spin the shaft faster than that
    between reads, this will misdetect wrap direction.
    """

    def __init__(self, pulse_nbr: int = ENC_PULSE_NBR):
        self.pulse_nbr = pulse_nbr
        self._last_raw = None
        self._revolutions = 0

    def update(self, raw_count: int) -> float:
        if self._last_raw is None:
            self._last_raw = raw_count
            return float(raw_count)

        delta = raw_count - self._last_raw
        half = self.pulse_nbr / 2

        if delta > half:
            # wrapped backward (raw jumped from near-0 to near-max)
            self._revolutions -= 1
        elif delta < -half:
            # wrapped forward (raw jumped from near-max to near-0)
            self._revolutions += 1

        self._last_raw = raw_count
        return raw_count + self._revolutions * self.pulse_nbr

    def reset(self):
        self._last_raw = None
        self._revolutions = 0


class MotorCANInterface:
    """
    Background-thread CAN interface.

    A daemon thread continuously reads incoming frames and updates the
    latest telemetry values under a lock. Command sends are synchronous
    and non-blocking (fire-and-forget, matching the no-ack protocol
    described).
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan"):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        self._unwrapper = EncoderUnwrapper()

        self.iq_readback = None
        self.elec_angle_deg = None
        self.enc_count_raw = None
        self.enc_count_unwrapped = None
        self.last_rx_time = None

        self._lock = threading.Lock()
        self._running = False
        self._rx_thread = None

    # ---- lifecycle ----
    def start_listening(self):
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def stop_listening(self):
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
        self.bus.shutdown()

    # ---- commands (Pi -> MCU) ----
    def send_start(self):
        self.bus.send(can.Message(arbitration_id=ID_START,
                      data=b"", is_extended_id=False))

    def send_stop(self):
        self.bus.send(can.Message(arbitration_id=ID_STOP,
                      data=b"", is_extended_id=False))

    def send_set_iq(self, amps: float):
        payload = struct.pack("<f", amps)
        self.bus.send(can.Message(arbitration_id=ID_SET_IQ,
                      data=payload, is_extended_id=False))

    # ---- telemetry (MCU -> Pi) ----
    def _rx_loop(self):
        while self._running:
            msg = self.bus.recv(timeout=0.5)
            if msg is None:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg: "can.Message"):
        with self._lock:
            if msg.arbitration_id == ID_IQ_READBACK and len(msg.data) >= 4:
                (self.iq_readback,) = struct.unpack("<f", msg.data[:4])
            elif msg.arbitration_id == ID_ELEC_ANGLE and len(msg.data) >= 2:
                (raw,) = struct.unpack("<h", msg.data[:2])
                self.elec_angle_deg = raw * DPP_TO_DEG
            elif msg.arbitration_id == ID_ENC_COUNT and len(msg.data) >= 4:
                (raw,) = struct.unpack("<I", msg.data[:4])
                self.enc_count_raw = raw
                self.enc_count_unwrapped = self._unwrapper.update(raw)
            self.last_rx_time = time.time()

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "iq_readback": self.iq_readback,
                "elec_angle_deg": self.elec_angle_deg,
                "enc_count_raw": self.enc_count_raw,
                "enc_count_unwrapped": self.enc_count_unwrapped,
                "last_rx_time": self.last_rx_time,
            }

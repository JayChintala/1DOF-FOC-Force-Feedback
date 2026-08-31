"""
CAN interface module for the 1DOF FOC Force-Feedback project.

Handles command TX (START / STOP / SET_IQ) and telemetry RX
(IQ_READBACK / ELEC_ANGLE / ENC_COUNT) over SocketCAN (can0, 1 Mbps).

Protocol (per node, base offset by CAN_NODE_STRIDE * node_index):
    base+0x001 START         Pi->MCU  no payload
    base+0x002 STOP          Pi->MCU  no payload
    base+0x003 SET_IQ        Pi->MCU  float32 LE, Amps
    base+0x010 IQ_READBACK    MCU->Pi float32 LE, Amps
    base+0x013 ELEC_ANGLE     MCU->Pi int16, DPP format
    base+0x014 ENC_COUNT      MCU->Pi uint32, raw TIM4 counter, wraps at M1_PULSE_NBR

ESC 1 uses node_base=0x000 (unchanged from the original single-ESC
protocol). ESC 2 uses node_base=0x020, matching the firmware-side
CAN_NODE_STRIDE in can_driver.h.

Known gotcha: SET_IQ is silently dropped by firmware unless the motor is
already in RUN state. Always START, wait briefly, then SET_IQ.
"""

import struct
import threading
import time

import can

# ---- CAN arbitration ID offsets (added to a node's base) ----
# Must match CAN_NODE_STRIDE and the offset macros in can_driver.h.
CAN_NODE_STRIDE = 0x020

OFFSET_START = 0x001
OFFSET_STOP = 0x002
OFFSET_SET_IQ = 0x003
OFFSET_IQ_READBACK = 0x010
OFFSET_ELEC_ANGLE = 0x013
OFFSET_ENC_COUNT = 0x014

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

# DPP_TO_DEG: conversion from the int16 ELEC_ANGLE "DPP" value to degrees.
# Signed 16-bit value spanning +-180 electrical degrees (32768 counts =
# 180 deg).
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



# ---- Shared bus ------------------------------------------------------------
# Subscribe to ELEC_ANGLE? Default off. Nothing in this repo reads
# elec_angle_deg (it's the FOC electrical angle, which wraps once per pole
# pair -- a commutation/alignment diagnostic, not a position source), and it
# is a third of the bus traffic. Left off, the kernel discards those frames
# before Python ever sees them. Flip to True if you need the field.
SUBSCRIBE_ELEC_ANGLE = False


class _SharedBus:
    """
    One SocketCAN socket and one RX thread for the whole process, shared by
    every MotorCANInterface on the same channel.

    WHY: each interface used to open its OWN unfiltered socket, so with two
    motors every frame on the bus was delivered twice and parsed twice in
    Python, ~12000 _handle_message calls/s under the GIL for 6000 frames/s of
    telemetry. telemetry_rate_probe.py measured the cost: 2.6% of frames
    handled more than 5 ms after they arrived, p99 ~33 ms, worst 48 ms, in
    bursts where the RX thread simply did not run -- plus ~0.2% lost to
    socket-queue overflow. The same probe measured one filtered socket on the
    same wire at a p99 of 0.27 ms with nothing over 5 ms.

    Two things fix it, both here:
      - one socket instead of one per motor, so each frame is parsed once;
      - kernel-side filters (can_filters) for only the IDs actually read, so
        the ~2000 frames/s of ELEC_ANGLE never cross into user space at all.

    Registration is dynamic: interfaces come and go with start_listening() /
    stop_listening(), and the filter set is recomputed each time. The socket
    opens on the first registration and closes on the last.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._buses = {}        # channel -> can.Bus
        self._nodes = {}        # channel -> {arb_id -> [interface, ...]}
        self._threads = {}      # channel -> Thread
        self._running = {}      # channel -> bool
        self._refcount = {}     # channel -> int

    def _filters_for(self, channel):
        return [{"can_id": arb_id, "can_mask": 0x7FF}
                for arb_id in sorted(self._nodes.get(channel, {}))]

    def register(self, iface, channel, bustype):
        """Subscribe iface's telemetry IDs and make sure the RX thread runs."""
        with self._lock:
            arb_ids = [iface._id_iq_readback, iface._id_enc_count]
            if SUBSCRIBE_ELEC_ANGLE:
                arb_ids.append(iface._id_elec_angle)

            table = self._nodes.setdefault(channel, {})
            for arb_id in arb_ids:
                table.setdefault(arb_id, []).append(iface)
            self._refcount[channel] = self._refcount.get(channel, 0) + 1

            if channel not in self._buses:
                self._buses[channel] = can.interface.Bus(
                    channel=channel, bustype=bustype,
                    can_filters=self._filters_for(channel),
                )
            else:
                # A later interface widens the filter set on the live socket.
                self._buses[channel].set_filters(self._filters_for(channel))

            if not self._running.get(channel):
                self._running[channel] = True
                th = threading.Thread(target=self._rx_loop, args=(channel,),
                                      daemon=True)
                self._threads[channel] = th
                th.start()
            return self._buses[channel]

    def unregister(self, iface, channel):
        """Drop iface's subscriptions; close the socket once nobody is left."""
        with self._lock:
            table = self._nodes.get(channel, {})
            for arb_id in list(table):
                table[arb_id] = [i for i in table[arb_id] if i is not iface]
                if not table[arb_id]:
                    del table[arb_id]
            self._refcount[channel] = max(0, self._refcount.get(channel, 0) - 1)
            if self._refcount[channel] > 0:
                if table:
                    self._buses[channel].set_filters(self._filters_for(channel))
                return
            self._running[channel] = False
            th = self._threads.pop(channel, None)
            bus = self._buses.pop(channel, None)
        # Join and shut down outside the lock: the RX thread takes it.
        if th is not None:
            th.join(timeout=1.0)
        if bus is not None:
            bus.shutdown()

    def send(self, channel, msg, bustype="socketcan"):
        with self._lock:
            bus = self._buses.get(channel)
            if bus is None:
                # Sending before start_listening() used to work, because the
                # socket was opened in MotorCANInterface.__init__. Preserve
                # that: open it here, send-only for now. A later register()
                # will widen the filters for whatever wants to receive.
                bus = can.interface.Bus(channel=channel, bustype=bustype,
                                        can_filters=self._filters_for(channel))
                self._buses[channel] = bus
        # One socket now serves every motor, so sends from different threads
        # are serialised rather than relying on per-frame write atomicity.
        with self._send_lock:
            bus.send(msg)

    def _rx_loop(self, channel):
        bus = self._buses[channel]
        while self._running.get(channel):
            try:
                msg = bus.recv(timeout=0.5)
            except Exception:
                if not self._running.get(channel):
                    break
                raise
            if msg is None:
                continue
            # Filters are kernel-side, so anything arriving here is wanted by
            # at least one interface. No per-frame dict copy, no lock: read a
            # snapshot of the subscriber list and dispatch.
            for iface in self._nodes.get(channel, {}).get(msg.arbitration_id, ()):
                iface._handle_message(msg)


_shared_bus = _SharedBus()


class MotorCANInterface:
    """
    CAN interface for one node (one motor). Public API unchanged:
    start_listening() / stop_listening() / send_start() / send_stop() /
    send_set_iq() / get_telemetry().

    The socket and RX thread are NOT per-instance any more -- they live in
    the process-wide _SharedBus above, which explains why. Telemetry values
    are still per-instance, updated under this instance's own lock.

    Command sends are synchronous and non-blocking (fire-and-forget,
    matching the no-ack protocol described).

    Each telemetry signal (IQ_READBACK, ELEC_ANGLE, ENC_COUNT) has its
    own timestamp (iq_readback_time / elec_angle_time / enc_count_time)
    so callers can tell which specific signal is stale, rather than
    relying on a single last_rx_time shared across all three.

    Those timestamps are taken when the RX thread PARSES the frame, which
    is not when the frame arrived -- see _SharedBus for the measured gap.
    enc_count_bus_time / iq_readback_bus_time carry the KERNEL's receive
    timestamp (msg.timestamp) instead, which is when the frame actually
    landed. Use those two for anything latency- or velocity-related, and the
    parse-time ones only for "is this link alive".

    Multiple instances (one per node_base) share the same physical bus and
    the same socket; each only sees its own node's IDs, because the dispatch
    table in _SharedBus is keyed by arbitration ID.
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan", node_base: int = 0x000):
        self._unwrapper = EncoderUnwrapper()
        self.node_base = node_base
        self._channel = channel
        self._bustype = bustype
        # Opened by start_listening() via the shared registry, not here --
        # the socket is process-wide, so it cannot belong to one instance.
        self.bus = None

        # Per-instance IDs, computed once from this node's base offset.
        self._id_start = node_base + OFFSET_START
        self._id_stop = node_base + OFFSET_STOP
        self._id_set_iq = node_base + OFFSET_SET_IQ
        self._id_iq_readback = node_base + OFFSET_IQ_READBACK
        self._id_elec_angle = node_base + OFFSET_ELEC_ANGLE
        self._id_enc_count = node_base + OFFSET_ENC_COUNT

        self.iq_readback = None
        self.elec_angle_deg = None
        self.enc_count_raw = None
        self.enc_count_unwrapped = None
        self.last_rx_time = None

        # Per-signal timestamps -- set only when that specific signal's
        # frame arrives, so staleness can be measured per-signal.
        self.iq_readback_time = None
        self.elec_angle_time = None
        self.enc_count_time = None

        # Kernel (SocketCAN) receive timestamps -- when the frame arrived,
        # as opposed to when this thread got around to parsing it. See the
        # class docstring: the difference is not small.
        self.iq_readback_bus_time = None
        self.enc_count_bus_time = None

        self._lock = threading.Lock()
        self._listening = False

    # ---- lifecycle ----
    def start_listening(self):
        if self._listening:
            return
        self.bus = _shared_bus.register(self, self._channel, self._bustype)
        self._listening = True

    def stop_listening(self):
        if not self._listening:
            return
        self._listening = False
        _shared_bus.unregister(self, self._channel)
        self.bus = None

    # ---- commands (Pi -> MCU) ----
    def send_start(self):
        _shared_bus.send(self._channel, can.Message(
            arbitration_id=self._id_start, data=b"", is_extended_id=False),
            self._bustype)

    def send_stop(self):
        _shared_bus.send(self._channel, can.Message(
            arbitration_id=self._id_stop, data=b"", is_extended_id=False),
            self._bustype)

    def send_set_iq(self, amps: float):
        payload = struct.pack("<f", amps)
        _shared_bus.send(self._channel, can.Message(
            arbitration_id=self._id_set_iq, data=payload, is_extended_id=False),
            self._bustype)

    # ---- telemetry (MCU -> Pi) ----
    def _handle_message(self, msg: "can.Message"):
        # Called from the shared RX thread, which only routes the IDs this
        # instance subscribed to. The per-ID checks below still stand, so the
        # method stays correct if it is ever handed an unrelated frame.
        with self._lock:
            now = time.time()
            if msg.arbitration_id == self._id_iq_readback and len(msg.data) >= 4:
                (self.iq_readback,) = struct.unpack("<f", msg.data[:4])
                self.iq_readback_time = now
                self.iq_readback_bus_time = msg.timestamp
            elif msg.arbitration_id == self._id_elec_angle and len(msg.data) >= 2:
                (raw,) = struct.unpack("<h", msg.data[:2])
                self.elec_angle_deg = raw * DPP_TO_DEG
                self.elec_angle_time = now
            elif msg.arbitration_id == self._id_enc_count and len(msg.data) >= 4:
                (raw,) = struct.unpack("<I", msg.data[:4])
                self.enc_count_raw = raw
                self.enc_count_unwrapped = self._unwrapper.update(raw)
                self.enc_count_time = now
                self.enc_count_bus_time = msg.timestamp
            else:
                return  # not this node's message -- don't update last_rx_time
            self.last_rx_time = now

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "iq_readback": self.iq_readback,
                "elec_angle_deg": self.elec_angle_deg,
                "enc_count_raw": self.enc_count_raw,
                "enc_count_unwrapped": self.enc_count_unwrapped,
                "last_rx_time": self.last_rx_time,
                "iq_readback_time": self.iq_readback_time,
                "elec_angle_time": self.elec_angle_time,
                "enc_count_time": self.enc_count_time,
                "iq_readback_bus_time": self.iq_readback_bus_time,
                "enc_count_bus_time": self.enc_count_bus_time,
            }

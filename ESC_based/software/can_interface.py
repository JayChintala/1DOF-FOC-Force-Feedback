"""
CAN interface module for the ESC_based firmware (1DOF FOC Force-Feedback).

Same transport as software/can_interface.py -- one shared, kernel-filtered
SocketCAN socket and one RX thread for every motor -- extended with the
ESC_based protocol: coupling configuration commands and the DBG frame.

Protocol (per node, base offset by CAN_NODE_STRIDE * node_index; must match
ESC_based/STM32/MCWorkbench/Inc/can_driver.h):
    base+0x001 START         Pi->MCU  no payload
    base+0x002 STOP          Pi->MCU  no payload (also cancels coupling)
    base+0x003 SET_IQ        Pi->MCU  float32 Iq, Amps. Direct mode only.
                                      Re-send at least every 200 ms or the
                                      firmware zeroes the current.
    base+0x004 COUPLE_GAINS  Pi->MCU  float32 Kp [A/count], float32 Kd [A/(count/s)]
    base+0x005 COUPLE_LOCAL  Pi->MCU  float32 Kd_local [A/(count/s)], float32 Iq_max [A]
    base+0x006 COUPLE_MODE   Pi->MCU  uint8 mode: 0 off, 1 hold, 2 peer;
                                      optional uint8 flags (bit 0 = predict)
    base+0x012 TELEM         MCU->Pi  8 bytes LE, once per 1 kHz firmware tick:
                               [0..3] float32 Iq, Amps (raw, unfiltered)
                               [4..5] uint16  raw TIM4 count, 0..3999
                               [6..7] uint16  MCU microsecond clock, wraps at 65536
    0x100+base+0x012 DBG     MCU->Pi  8 bytes LE, what the ESC's controller did:
                               [0..1] int16   Iq commanded, mA
                               [2..3] int16   coupling error, counts
                               [4..5] uint16  MCU microsecond clock -- the SAME
                                              value as that tick's TELEM
                               [6]    uint8   status bits (STATUS_* below)
                               [7]    uint8   peer TELEM age, 0.1 ms (255 = >=25.5 ms)
                             Every tick while coupling is requested/engaged,
                             10 Hz otherwise.

The TELEM layout is unchanged, so this module still works against the
original firmware for everything except the coupling commands and DBG.

ESC 1 uses node_base=0x000, ESC 2 node_base=0x020.
"""

import collections
import os
import socket
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
OFFSET_COUPLE_GAINS = 0x004
OFFSET_COUPLE_LOCAL = 0x005
OFFSET_COUPLE_MODE = 0x006
OFFSET_TELEM = 0x012
# DBG sits 0x100 above TELEM, outside every node's block, so it loses
# arbitration to all control traffic -- see CAN_ID_DBG in can_driver.h.
OFFSET_DBG = 0x100 + OFFSET_TELEM

COUPLE_MODE_OFF = 0
COUPLE_MODE_HOLD = 1
COUPLE_MODE_PEER = 2
COUPLE_FLAG_PREDICT = 1 << 0  # PEER mode: couple to the peer's predicted
                              # position now, not its ~1 ms-old sample

# DBG status byte -- COUPLE_STATUS_* in couple_ctrl.h.
STATUS_RUN = 1 << 0        # motor state machine in RUN
STATUS_ENGAGED = 1 << 1    # coupling law driving Iq
STATUS_REQUESTED = 1 << 2  # a mode is requested (maybe not engaged yet)
STATUS_MODE_PEER = 1 << 3  # requested mode is PEER (else HOLD)
STATUS_SATURATED = 1 << 4  # Iq clamped at Iq_max
STATUS_TRIP_SHIFT = 5      # bits 5..7: last trip code

TRIP_NAMES = {
    0: "none",
    1: "peer TELEM timeout",
    2: "error limit (10 rev)",
    3: "velocity limit (50 rev/s)",
    4: "SET_IQ host timeout",
    5: "motor left RUN (fault?)",
}

# Peer age byte saturates here: "25.5 ms or more, or never received".
PEER_AGE_SATURATED_MS = 25.5

# ENC_PULSE_NBR: wrap modulus of the raw TIM4 encoder counter.
# Confirmed from MCWorkbench\Src\mc_config_common.c:
#   .PulseNumber = M1_ENCODER_PPR * 4
# with M1_ENCODER_PPR = 1000 (pmsm_motor_parameters.h) -> 4000. TIM4 counts
# 0..3999 (ARR = M1_PULSE_NBR = 3999), so 4000 is the modulus.
ENC_PULSE_NBR = 4000

# Modulus of the MCU's microsecond timestamp field (uint16). It wraps every
# 65.536 ms against a 1 ms send period, so consecutive samples are never
# ambiguous -- the same unwrap the encoder count gets. (ESC_based fixes a
# firmware bug where this field also jumped back ~32 ms every 25.26 s, when
# the DWT cycle counter wrapped; see MicroClock in can_driver.c.)
MCU_CLOCK_MODULUS = 1 << 16

# Records not yet drained by drain_records() are capped here per motor so a
# script that never drains cannot grow without bound: 120 s at 1 kHz.
RECORD_BUFFER_MAX = 120_000
# ------------------------------------------------------------------------


def decode_status(flags):
    """DBG status byte -> dict of named fields."""
    return {
        "run": bool(flags & STATUS_RUN),
        "engaged": bool(flags & STATUS_ENGAGED),
        "requested": bool(flags & STATUS_REQUESTED),
        "mode_peer": bool(flags & STATUS_MODE_PEER),
        "saturated": bool(flags & STATUS_SATURATED),
        "trip": (flags >> STATUS_TRIP_SHIFT) & 0x7,
    }


class WrappingCounter:
    """
    Converts a counter that wraps at a fixed modulus into a continuous
    value, by tracking wraparounds between consecutive samples.

    Used twice: for the raw encoder count (modulus ENC_PULSE_NBR) and for
    the MCU's uint16 microsecond clock (modulus MCU_CLOCK_MODULUS). Both
    are the same problem.

    Assumes consecutive samples never differ by more than half the modulus
    (i.e. you're sampling fast enough relative to how fast the counter
    moves). For the encoder that means not spinning the shaft faster than
    half a revolution per sample; for the microsecond clock it means a send
    period well under 32.768 ms, which at 1 kHz it is by a factor of 32.
    """

    def __init__(self, modulus: int = ENC_PULSE_NBR):
        self.modulus = modulus
        self._last_raw = None
        self._revolutions = 0

    def update(self, raw_count: int) -> float:
        if self._last_raw is None:
            self._last_raw = raw_count
            return float(raw_count)

        delta = raw_count - self._last_raw
        half = self.modulus / 2

        if delta > half:
            # wrapped backward (raw jumped from near-0 to near-max)
            self._revolutions -= 1
        elif delta < -half:
            # wrapped forward (raw jumped from near-max to near-0)
            self._revolutions += 1

        self._last_raw = raw_count
        return raw_count + self._revolutions * self.modulus

    def reset(self):
        self._last_raw = None
        self._revolutions = 0


# ---- Shared bus ------------------------------------------------------------
# RX socket buffer, bytes. The kernel default is ~208 KB, which at 2000
# frames/s is many seconds of slack -- until the RX thread is descheduled in
# a burst, which telemetry_rate_probe.py measured happening (p99 ~33 ms,
# worst 48 ms, ~0.2% of frames lost to socket-queue overflow). Overflow is
# silent and drops the OLDEST frames, so it corrupts velocity estimates
# rather than announcing itself. Buying headroom here is far cheaper than
# the alternative.
#
# The kernel doubles whatever SO_RCVBUF asks for (bookkeeping overhead), so
# this requests 2 MB and yields ~4 MB -- which is rmem_max on this host.
# Above rmem_max the request is silently clamped, not refused; raising it
# further needs SO_RCVBUFFORCE and CAP_NET_ADMIN, which is not worth it.
RX_BUFFER_BYTES = 2 * 1024 * 1024

# SCHED_FIFO priority for the RX thread. Low enough to sit under anything the
# kernel runs at 50+, high enough to preempt every SCHED_OTHER task on the
# box. With the coupling on the ESCs the Pi is only logging, so this no
# longer protects a control loop -- it protects the log from gaps.
# Best-effort; see _apply_rx_thread_priority().
RX_THREAD_RT_PRIORITY = 20


def _enlarge_rx_buffer(bus, nbytes: int = RX_BUFFER_BYTES):
    """
    Raise SO_RCVBUF on the SocketCAN socket. Best-effort and non-fatal.

    python-can exposes the raw socket as .socket on the socketcan backend
    only; on any other backend (virtual buses in tests, slcan, a USB
    adapter) there is nothing to tune and nothing to complain about.
    """
    sock = getattr(bus, "socket", None)
    if sock is None:
        return None
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, nbytes)
        return sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    except OSError:
        # Clamped or refused by the kernel. The default buffer still works,
        # it just has less headroom -- not worth failing a run over.
        return None


def _apply_rx_thread_priority(priority: int = RX_THREAD_RT_PRIORITY):
    """
    Put the calling thread on SCHED_FIFO. Best-effort: silently does nothing
    without CAP_SYS_NICE (or a matching RTPRIO rlimit), and nothing at all
    off Linux.
    """
    setter = getattr(os, "sched_setscheduler", None)
    if setter is None:
        return False
    try:
        setter(0, os.SCHED_FIFO, os.sched_param(priority))
        return True
    except (OSError, PermissionError, AttributeError):
        return False


class _SharedBus:
    """
    One SocketCAN socket and one RX thread for the whole process, shared by
    every MotorCANInterface on the same channel, with kernel-side filters
    for only the IDs actually read. See software/can_interface.py for the
    measurements behind this design (one socket per motor parsed every frame
    twice and stalled the RX thread for tens of milliseconds).

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
            arb_ids = [iface._id_telem, iface._id_dbg]

            table = self._nodes.setdefault(channel, {})
            for arb_id in arb_ids:
                table.setdefault(arb_id, []).append(iface)
            self._refcount[channel] = self._refcount.get(channel, 0) + 1

            if channel not in self._buses:
                self._buses[channel] = can.interface.Bus(
                    channel=channel, interface=bustype,
                    can_filters=self._filters_for(channel),
                )
                _enlarge_rx_buffer(self._buses[channel])
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
                # Send-only until something registers to receive.
                bus = can.interface.Bus(channel=channel, interface=bustype,
                                        can_filters=self._filters_for(channel))
                self._buses[channel] = bus
        # One socket serves every motor, so sends from different threads
        # are serialised rather than relying on per-frame write atomicity.
        with self._send_lock:
            bus.send(msg)

    def _rx_loop(self, channel):
        _apply_rx_thread_priority()
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
            for iface in self._nodes.get(channel, {}).get(msg.arbitration_id, ()):
                iface._handle_message(msg)


_shared_bus = _SharedBus()


class MotorCANInterface:
    """
    CAN interface for one node (one ESC).

    Two ways to read it:
      get_telemetry() / get_debug()  latest values, for status displays
      drain_records()                every 1 kHz tick exactly once, for logs

    A record is one TELEM frame joined with the DBG frame of the same tick,
    matched on the MCU timestamp both carry. When DBG is not being sent
    every tick (no coupling requested), records carry TELEM fields only and
    the DBG fields are None.

    Velocity should be differentiated against mcu_time_us, never against
    arrival times -- see software/can_interface.py for the measurements.
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan",
                 node_base: int = 0x000):
        self._unwrapper = WrappingCounter(ENC_PULSE_NBR)
        self._clock_unwrapper = WrappingCounter(MCU_CLOCK_MODULUS)
        self.node_base = node_base
        self._channel = channel
        self._bustype = bustype
        self.bus = None

        self._id_start = node_base + OFFSET_START
        self._id_stop = node_base + OFFSET_STOP
        self._id_set_iq = node_base + OFFSET_SET_IQ
        self._id_couple_gains = node_base + OFFSET_COUPLE_GAINS
        self._id_couple_local = node_base + OFFSET_COUPLE_LOCAL
        self._id_couple_mode = node_base + OFFSET_COUPLE_MODE
        self._id_telem = node_base + OFFSET_TELEM
        self._id_dbg = node_base + OFFSET_DBG

        # Latest TELEM.
        self.iq_readback = None
        self.enc_count_raw = None
        self.enc_count_unwrapped = None
        self.mcu_time_us = None
        self.last_rx_time = None
        self.enc_count_bus_time = None

        # Latest DBG.
        self.dbg_iq_cmd = None
        self.dbg_err = None
        self.dbg_flags = None
        self.dbg_peer_age_ms = None
        self.dbg_time = None

        # TELEM waiting for its DBG; emitted on the matching DBG or when the
        # next TELEM shows none is coming.
        self._pending = None
        self._records = collections.deque(maxlen=RECORD_BUFFER_MAX)
        self.unmatched_dbg = 0

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
    def _send(self, arb_id, data=b""):
        _shared_bus.send(self._channel, can.Message(
            arbitration_id=arb_id, data=data, is_extended_id=False),
            self._bustype)

    def send_start(self):
        self._send(self._id_start)

    def send_stop(self):
        self._send(self._id_stop)

    def send_set_iq(self, amps: float):
        self._send(self._id_set_iq, struct.pack("<f", amps))

    def send_couple_gains(self, kp: float, kd: float):
        """Kp in A/count, Kd in A/(count/s). Negative or NaN is ignored by the ESC."""
        self._send(self._id_couple_gains, struct.pack("<ff", kp, kd))

    def send_couple_local(self, kd_local: float, iq_max: float):
        """Kd_local in A/(count/s); Iq_max in A, capped at 0.8 A by the ESC."""
        self._send(self._id_couple_local, struct.pack("<ff", kd_local, iq_max))

    def send_couple_mode(self, mode: int, flags: int = 0):
        """COUPLE_MODE_OFF / _HOLD / _PEER, plus COUPLE_FLAG_* bits.
        Re-sending a mode re-zeroes it."""
        self._send(self._id_couple_mode, struct.pack("<BB", mode, flags))

    # ---- telemetry (MCU -> Pi) ----
    def _handle_message(self, msg: "can.Message"):
        # Called from the shared RX thread. Length is checked strictly: a
        # short frame has no valid interpretation, since the fields sit at
        # fixed offsets.
        if len(msg.data) != 8:
            return
        if msg.arbitration_id == self._id_telem:
            self._handle_telem(msg)
        elif msg.arbitration_id == self._id_dbg:
            self._handle_dbg(msg)

    def _handle_telem(self, msg):
        iq, enc_raw, mcu_us = struct.unpack("<fHH", msg.data)
        with self._lock:
            now = time.time()
            self.iq_readback = iq
            self.enc_count_raw = enc_raw
            self.enc_count_unwrapped = self._unwrapper.update(enc_raw)
            self.mcu_time_us = self._clock_unwrapper.update(mcu_us)
            self.last_rx_time = now
            self.enc_count_bus_time = msg.timestamp

            if self._pending is not None:
                self._records.append(self._pending)  # no DBG this tick
            self._pending = {
                "bus_time": msg.timestamp,
                "mcu_us_raw": mcu_us,
                "mcu_time_us": self.mcu_time_us,
                "enc": self.enc_count_unwrapped,
                "iq_meas": iq,
                "iq_cmd": None, "err": None, "flags": None, "peer_age_ms": None,
            }

    def _handle_dbg(self, msg):
        iq_ma, err, mcu_us, flags, age = struct.unpack("<hhHBB", msg.data)
        iq_cmd = iq_ma / 1000.0
        age_ms = age / 10.0
        with self._lock:
            self.dbg_iq_cmd = iq_cmd
            self.dbg_err = err
            self.dbg_flags = flags
            self.dbg_peer_age_ms = age_ms
            self.dbg_time = time.time()

            pending = self._pending
            if pending is not None and pending["mcu_us_raw"] == mcu_us:
                pending.update(iq_cmd=iq_cmd, err=err, flags=flags,
                               peer_age_ms=age_ms)
                self._records.append(pending)
                self._pending = None
            else:
                # Its TELEM never arrived (dropped on the ESC's Tx FIFO, or
                # lost here). Counted, not guessed at.
                self.unmatched_dbg += 1

    def drain_records(self) -> list:
        """Every record completed since the last call, oldest first."""
        with self._lock:
            out = list(self._records)
            self._records.clear()
        return out

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "iq_readback": self.iq_readback,
                "enc_count_raw": self.enc_count_raw,
                "enc_count_unwrapped": self.enc_count_unwrapped,
                "mcu_time_us": self.mcu_time_us,
                "last_rx_time": self.last_rx_time,
                "enc_count_bus_time": self.enc_count_bus_time,
            }

    def get_debug(self) -> dict:
        with self._lock:
            out = {
                "iq_cmd": self.dbg_iq_cmd,
                "err": self.dbg_err,
                "flags": self.dbg_flags,
                "peer_age_ms": self.dbg_peer_age_ms,
                "dbg_time": self.dbg_time,
            }
        if out["flags"] is not None:
            out.update(decode_status(out["flags"]))
        return out

"""
CAN interface module for the 1DOF FOC Force-Feedback project.

Handles command TX (START / STOP / SET_IQ) and telemetry RX (one merged
TELEM frame) over SocketCAN (can0, 1 Mbps).

Protocol (per node, base offset by CAN_NODE_STRIDE * node_index):
    base+0x001 START   Pi->MCU  no payload
    base+0x002 STOP    Pi->MCU  no payload
    base+0x003 SET_IQ  Pi->MCU  float32 LE, Amps
    base+0x012 TELEM   MCU->Pi  8 bytes LE, once per 1 kHz firmware tick:
                         [0..3] float32 Iq, Amps (raw, unfiltered)
                         [4..5] uint16  raw TIM4 count, 0..3999
                         [6..7] uint16  MCU microsecond clock, wraps at 65536

TELEM replaced three separate frames (IQ_READBACK 0x010, IQ_MEAN 0x011,
ENC_COUNT 0x014, plus the earlier standalone ELEC_ANGLE 0x013). Two motors
at 1 kHz went from 6000 frames/s to 2000, and from three Pi-side interrupts
per tick to one. The dropped signals -- the Iq EWMA, the Iq boxcar mean and
the electrical angle -- had no readers anywhere in this directory; they were
being generated, transmitted, parsed and discarded.

The merge also fixed something the frame-count saving is incidental to:
Iq and position now share one timestamp by construction. They used to
arrive in separate frames whose relative delay nothing bounded, so any
controller reading current and position together carried a skew it could
neither measure nor correct.

ESC 1 uses node_base=0x000 (unchanged from the original single-ESC
protocol). ESC 2 uses node_base=0x020, matching the firmware-side
CAN_NODE_STRIDE in can_driver.h.

Known gotcha: SET_IQ is silently dropped by firmware unless the motor is
already in RUN state. Always START, wait briefly, then SET_IQ.
"""

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
OFFSET_TELEM = 0x012
# 0x010 (IQ_READBACK), 0x011 (IQ_MEAN), 0x013 (ELEC_ANGLE) and 0x014
# (ENC_COUNT) are all retired -- see the module docstring. TELEM deliberately
# does NOT reuse 0x010: the old 0x010 payload also started with a float32 Iq,
# so a node left on pre-merge firmware would parse as valid here while its
# former EWMA bytes were read as an encoder count and a timestamp. A fresh ID
# makes a half-flashed bus fall silent instead, which is a failure you notice.

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

# Modulus of the MCU's microsecond timestamp field (uint16). It wraps every
# 65.536 ms against a 1 ms send period, so consecutive samples are never
# ambiguous -- the same unwrap the encoder count gets.
MCU_CLOCK_MODULUS = 1 << 16
# ------------------------------------------------------------------------


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
# There used to be a SUBSCRIBE_ELEC_ANGLE flag here, then a note explaining
# that the angle had been folded into IQ_MEAN so the flag was unnecessary.
# Both are gone: the electrical angle is not transmitted at all any more.
# Nothing read it, and a signal nothing reads still costs a Tx slot on the
# MCU, bus bandwidth, an interrupt and a parse on the Pi.

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
# box -- which is the point: the thread only has to run for a few
# microseconds per frame, and the damage comes entirely from it not being
# scheduled promptly. Best-effort; see _apply_rx_thread_priority().
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

    WHY: every microsecond between a frame landing in the socket queue and
    this thread parsing it is jitter on the timestamps the control loop
    reads. Under SCHED_OTHER the thread competes with the control loop and
    with everything else on the box, and the measured result was bursts
    where it simply did not run for tens of milliseconds.

    This is the half of the fix that lives in the process. The other half --
    steering the CAN IRQ onto a core the control loop is not using -- is
    system configuration, not something a library can do to itself; see
    setup_can_realtime.sh.
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
        frames for IDs this process does not read never cross into user
        space at all.

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
            arb_ids = [iface._id_telem]

            table = self._nodes.setdefault(channel, {})
            for arb_id in arb_ids:
                table.setdefault(arb_id, []).append(iface)
            self._refcount[channel] = self._refcount.get(channel, 0) + 1

            if channel not in self._buses:
                self._buses[channel] = can.interface.Bus(
                    channel=channel, bustype=bustype,
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

    Iq and the encoder count now arrive in ONE frame, so the per-signal
    timestamps that used to distinguish them (iq_readback_time vs
    enc_count_time) are necessarily equal. Both names are kept, because a
    pile of scripts in this directory read one or the other, and because
    "which signal is stale" is still a meaningful question if a third
    signal is ever added back.

    THREE CLOCKS, in increasing order of how much you should trust them for
    anything involving a derivative:

      *_time          when the RX thread PARSED the frame. Scheduling noise
                      on top of everything below it. Use only for "is this
                      link alive".
      *_bus_time      the KERNEL's receive timestamp (msg.timestamp) -- when
                      the frame actually landed. Free of GIL and thread
                      scheduling, but still carries Tx FIFO wait, bus
                      arbitration and IRQ latency.
      mcu_time_us     the MCU's own microsecond clock, sampled in the same
                      breath as Iq and the encoder count. Immune to the
                      transport entirely.

    Velocity should be differentiated against mcu_time_us. A 1 ms nominal
    period with a transport whose p99 jitter is 0.27 ms means dt taken from
    arrival times is wrong by up to ~27%, and that error lands directly on
    the derivative, where a controller's D term amplifies it. dt from
    mcu_time_us is exact regardless of what the transport did -- and stays
    exact if the link is ever moved to UART.

    Multiple instances (one per node_base) share the same physical bus and
    the same socket; each only sees its own node's IDs, because the dispatch
    table in _SharedBus is keyed by arbitration ID.
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan", node_base: int = 0x000):
        self._unwrapper = WrappingCounter(ENC_PULSE_NBR)
        self._clock_unwrapper = WrappingCounter(MCU_CLOCK_MODULUS)
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
        self._id_telem = node_base + OFFSET_TELEM

        # Raw, unfiltered Iq in Amps, sampled once per firmware tick. The
        # 16 kHz EWMA and boxcar mean the firmware also computes are no
        # longer transmitted -- nothing here read them. If raw Iq turns out
        # too noisy to control on, the fix is one line in CAN_SendTelemetry()
        # (send IqTelem_GetEwma() instead) and nothing on this side.
        self.iq_readback = None
        self.enc_count_raw = None
        self.enc_count_unwrapped = None

        # MCU microsecond clock, unwrapped past the uint16 rollover into a
        # continuous count. Arbitrary epoch -- only differences are
        # meaningful, which is all a dt needs.
        self.mcu_time_us = None

        self.last_rx_time = None

        # Parse-time stamps. Equal to each other now that one frame carries
        # both signals; kept as separate names so existing callers work.
        self.iq_readback_time = None
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
        # instance subscribed to. The ID check below still stands, so the
        # method stays correct if it is ever handed an unrelated frame.
        #
        # Length is checked strictly rather than with the >= that the old
        # multi-frame parser used. That leniency existed to stay compatible
        # with a node on older firmware sending a shorter frame; it cannot
        # serve that purpose here, because a short TELEM frame has no valid
        # interpretation -- the encoder count and timestamp are at fixed
        # offsets at the END of the payload, so a truncated frame would
        # silently yield a stale position rather than a detectably absent
        # one. Better to ignore it.
        if msg.arbitration_id != self._id_telem or len(msg.data) != 8:
            return

        iq, enc_raw, mcu_us = struct.unpack("<fHH", msg.data)

        with self._lock:
            now = time.time()
            self.iq_readback = iq
            self.enc_count_raw = enc_raw
            self.enc_count_unwrapped = self._unwrapper.update(enc_raw)
            self.mcu_time_us = self._clock_unwrapper.update(mcu_us)

            # One frame, so these are the same instant by construction --
            # which is the whole point of the merge.
            self.iq_readback_time = now
            self.enc_count_time = now
            self.iq_readback_bus_time = msg.timestamp
            self.enc_count_bus_time = msg.timestamp
            self.last_rx_time = now

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "iq_readback": self.iq_readback,
                "enc_count_raw": self.enc_count_raw,
                "enc_count_unwrapped": self.enc_count_unwrapped,
                # Differentiate position against this, not against the
                # arrival times below. See the class docstring.
                "mcu_time_us": self.mcu_time_us,
                "last_rx_time": self.last_rx_time,
                "iq_readback_time": self.iq_readback_time,
                "enc_count_time": self.enc_count_time,
                "iq_readback_bus_time": self.iq_readback_bus_time,
                "enc_count_bus_time": self.enc_count_bus_time,
            }

---
name: can-hat-irq-drops
description: Waveshare MCP2518FD HAT (swapped in 2026-09-22) costs one IRQ+SPI read per CAN frame on CPU0, dropping ~1.7% of frames and destabilising the PD/teleop loops
metadata:
  type: project
---

Swapping the gs_usb adapter for the Waveshare 2-CH isolated CAN FD HAT
(MCP2518FD on spi0.0) on 2026-09-22 destabilised every PD loop. Measured
2026-09-22: physical layer clean (berr-counter 0, bus-errors 0, arbit-lost 0,
bus-off 0) but `rx_dropped` 3.67M / 217M frames (1.69%) and IRQ 59 (spi0.0,
GPIO25) at ~1 interrupt per CAN frame, all on CPU0 alongside the Python RX
thread. gs_usb baseline was rx_dropped 0.

**Why:** python-can uses SO_TIMESTAMPNS (software stamp at socket queueing),
NOT the MCP2518FD hardware timestamp, so `enc_count_bus_time` is now stamped
after a contended SPI read chain. teleop divides position deltas by that value
for velocity, and position_hold divides by the loop period, so dropped samples
read as 2x velocity spikes. Both feed KD / KV_SLOPE / the tau_h observer.

**How to apply:** fix timing before re-tuning gains. Cut frames/s first (stop
sending ELEC_ANGLE, which nothing reads but still costs an IRQ; pack
iq float32 + enc uint32 into one 8-byte frame), then move IRQ 59 off the
control-loop core and raise SO_RCVBUF. Best long-term fix is a firmware µs
timestamp in the telemetry payload, which makes velocity transport-independent.
Related: [[flashed-telemetry-mismatch]], [[esc1-bus-dropouts-undervoltage]].

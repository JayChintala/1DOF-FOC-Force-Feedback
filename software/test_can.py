import can
import time

bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Send a ping - read Iq (message 0x014, node 0)
arb_id = (0 << 5) | 0x14
msg = can.Message(
    arbitration_id=arb_id,
    data=[],
    is_remote_frame=True,
    is_extended_id=False
)

print("Sending Iq request to Node 0...")
bus.send(msg)

response = bus.recv(timeout=2.0)
if response:
    print(f"Got response: {response}")
else:
    print("No response - check CAN wiring and termination")

bus.shutdown()

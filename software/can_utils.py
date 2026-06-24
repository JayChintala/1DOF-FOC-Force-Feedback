import can
import struct

def read_Iq(bus, node_id):
    """Read Iq current from motor via CAN. Returns float in Amps."""
    # ODrive CAN protocol: Get_Iq is message 0x014
    arb_id = (node_id << 5) | 0x14
    msg = can.Message(arbitration_id=arb_id, data=[], is_remote_frame=True, is_extended_id=False)
    bus.send(msg)
    
    # Wait for response
    response = bus.recv(timeout=0.1)
    if response is None:
        return 0.0
    
    # Iq is second float in the response (bytes 4-7)
    iq = struct.unpack_from('<f', response.data, 4)[0]
    return iq

def send_torque(bus, node_id, torque):
    """Send torque command to motor via CAN."""
    # ODrive CAN protocol: Set_Input_Torque is message 0x00E
    arb_id = (node_id << 5) | 0x0E
    data = struct.pack('<f', torque)
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)
    
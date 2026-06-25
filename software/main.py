import can
import time
from config import Kt, r, CAN_CHANNEL, CAN_BITRATE, NODE_SENSE, NODE_REPLICATE, DT
from can_utils import read_Iq, send_torque

def main():
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    
    print("Taring force sensor...")
    time.sleep(0.5)
    Iq_offset = read_Iq(bus, NODE_SENSE)
    print(f"Iq offset: {Iq_offset:.4f} A")

    print("Starting force loop at 500Hz. Ctrl+C to stop.")
    try:
        while True:
            Iq = read_Iq(bus, NODE_SENSE) - Iq_offset
            torque = Kt * Iq
            F = torque / r
            tau_cmd = F * r  # = torque, but explicit for clarity
            send_torque(bus, NODE_REPLICATE, tau_cmd)
            time.sleep(DT)

    except KeyboardInterrupt:
        print("Stopping...")
        send_torque(bus, NODE_REPLICATE, 0.0)
        bus.shutdown()

if __name__ == "__main__":
    main()

# Motor / system constants
Kt = 10.73        # Nm/A — will calibrate empirically
r = 0.015         # spool radius in meters — measure precisely

# CAN
CAN_CHANNEL = 'can0'
CAN_BITRATE = 1000000
NODE_SENSE = 0     # Motor 1 — force sensing
NODE_REPLICATE = 1 # Motor 2 — force replication

# Loop
LOOP_HZ = 500
DT = 1.0 / LOOP_HZ

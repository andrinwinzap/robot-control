import serial
from Trajectory import *
from SerialParser import *

PORT = '/dev/ttyUSB0'
BAUD = 115200


ser = serial.Serial(PORT, BAUD, timeout=0.1)
com = SerialProtocol(ser)

def generate_test_trajectory(n=50):
    wps = []
    for i in range(n):
        wps.append(Waypoint(position=i*0.01, velocity=i*0.02, timestamp=i*20))
    return Trajectory(wps)

cmd = 0x04
traj = generate_test_trajectory(63)
print("Trajectory contents:")
print(traj)
payload = traj.serialize()
com.send_packet(cmd, payload)


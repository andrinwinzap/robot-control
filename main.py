import serial
import time
from Trajectory import *
from SerialParser import *
from Serialization import *

PORT = '/dev/ttyUSB0'
BAUD = 115200
TIMEOUT = 1

PING = 0x01
HOME = 0x02
POS  = 0x03
LOAD_TRAJ = 0x04
EXEC_TRAJ = 0x05
FINISHED = 0x06
ACK  = 0xEE
NACK = 0xFF

ser = serial.Serial(PORT, BAUD, timeout=0.1)
com = SerialProtocol(ser)

def pos():
    com.send_packet(POS)
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    return read_float_le(cmd.payload)

def ping():
    com.send_packet(PING)
    while not com.available():
        start = time.time()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([ACK]):
        return True
    else:
        return False
    
def home():
    com.send_packet(HOME)
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd != bytes([ACK]):
        print("ACK not received")
        return False
    print("ACKED")
    while not com.available():
        pass
    cmd = com.read()
    if cmd.cmd == bytes([FINISHED]):
        return True
    else:
        return False
    
    
def load_traj(trajectory: Trajectory):
    com.send_packet(LOAD_TRAJ, trajectory.serialize())
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([ACK]):
        return True
    else:
        return False
    
def exec_traj():
    com.send_packet(EXEC_TRAJ)
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([ACK]):
        return True
    else:
        return False
    
# print(pos())
# print(ping())

# wps = []
# for i in range(64):
#     wps.append(Waypoint(position=i*0.01, velocity=i*0.02, timestamp=i*20))
# trajectory = Trajectory(wps)

# print(load_traj(trajectory))
# print(exec_traj())


print(home())   
wps = [Waypoint(0, 0, 0), Waypoint(3.14/2, 0, 5000), Waypoint(0, 0, 10000)]
traj = Trajectory(wps)
load_traj(traj)
exec_traj()
import serial
import time
from Trajectory import *
from SerialParser import *
from Serialization import *

PORT = '/dev/ttyUSB1'
BAUD = 115200
TIMEOUT = 1

class CommandByte:
    PING = 0x01
    HOME = 0x02
    POS  = 0x03
    LOAD_TRAJ = 0x04
    EXEC_TRAJ = 0x05
    FINISHED = 0x06
    STATUS = 0x07
    ACK  = 0xEE
    NACK = 0xFF

class StatusByte:
    IDLE = 0x01
    HOMING = 0x02
    EXECUTING_TRAJ = 0x03

ser = serial.Serial(PORT, BAUD, timeout=0.1)
com = SerialProtocol(ser)

def pos():
    com.send_packet(CommandByte.POS)
    start = time.time()
    while True:
        if time.time() - start > TIMEOUT:
            return False
        if com.available():
            cmd = com.read()
            if cmd.cmd == bytes([CommandByte.POS]):
                break
    return read_float_le(cmd.payload)

def ping():
    com.send_packet(CommandByte.PING)
    while not com.available():
        start = time.time()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([CommandByte.ACK]):
        return True
    else:
        return False
    
def home():
    com.send_packet(CommandByte.HOME)
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd != bytes([CommandByte.ACK]):
        print("ACK not received")
        return False
    print("ACKED")
    while True:
        com.send_packet(CommandByte.STATUS)
        start = time.time()
        while not com.available():
            if time.time() - start > TIMEOUT:
                return False
        cmd = com.read()
        if cmd.cmd == bytes([CommandByte.STATUS]):
            if cmd.payload[0] == StatusByte.IDLE:
                return True
    
    
def load_traj(trajectory: Trajectory):
    com.send_packet(CommandByte.LOAD_TRAJ, trajectory.serialize())
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([CommandByte.ACK]):
        return True
    else:
        return False
    
def exec_traj():
    com.send_packet(CommandByte.EXEC_TRAJ)
    start = time.time()
    while not com.available():
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd != bytes([CommandByte.ACK]):
        print("ACK not received")
        return False
    print("ACKED")
    while True:
        com.send_packet(CommandByte.STATUS)
        start = time.time()
        while not com.available():
            if time.time() - start > TIMEOUT:
                return False
        cmd = com.read()
        if cmd.cmd == bytes([CommandByte.STATUS]):
            print(cmd.payload[0])
            if cmd.payload[0] == StatusByte.IDLE:
                return True
    
print(home())
wps = [Waypoint(0, 0, 0), Waypoint(3.14/2, 0, 5000), Waypoint(0, 0, 10000)]
traj = Trajectory(wps)
print(load_traj(traj))
print(exec_traj())

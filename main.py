import serial
import time
from Trajectory import *
from SerialProtocol import *
from Serialization import *

PORT = '/dev/ttyUSB0'
BAUD = 115200
TIMEOUT = 1

ser = serial.Serial(PORT, BAUD, timeout=0.1)

def write_callback(packet):
    ser.write(packet)

com = SerialProtocol(Bytes.Address.MASTER, write_callback)


def parse_serial():
    while ser.in_waiting > 0:
        byte = ser.read(1)
        if byte:
            com.feed(byte[0])

def pos():
    com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.POS)
    start = time.time()
    while True:
        parse_serial()
        if time.time() - start > TIMEOUT:
            return False
        if com.available():
            cmd = com.read()
            if cmd.cmd == bytes([Bytes.Command.POS]):
                break
    return read_float_le(cmd.payload)

def ping():
    com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.PING)
    while not com.available():
        parse_serial()
        start = time.time()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([Bytes.Command.ACK]):
        return True
    else:
        return False

def estop():
    com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.ESTOP)
    while not com.available():
        parse_serial()
        start = time.time()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([Bytes.Command.ACK]):
        return True
    else:
        return False
    
def home():
    com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.HOME)
    start = time.time()
    while not com.available():
        parse_serial()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd != bytes([Bytes.Command.ACK]):
        print("ACK not received")
        return False
    print("ACKED")
    while True:
        com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.STATUS)
        start = time.time()
        while not com.available():
            parse_serial()
            if time.time() - start > TIMEOUT:
                return False
        cmd = com.read()
        if cmd.cmd == bytes([Bytes.Command.STATUS]):
            if cmd.payload[0] == Bytes.Status.IDLE:
                return True
    
    
def load_traj(trajectory: ActuatorTrajectory):
    com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.LOAD_TRAJ, trajectory.serialize())
    start = time.time()
    while not com.available():
        parse_serial()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd == bytes([Bytes.Command.ACK]):
        return True
    else:
        return False
    
def exec_traj():
    com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.EXEC_TRAJ)
    start = time.time()
    while not com.available():
        parse_serial()
        if time.time() - start > TIMEOUT:
            return False
    cmd = com.read()
    if cmd.cmd != bytes([Bytes.Command.ACK]):
        print("ACK not received")
        return False
    # print("ACKED")
    # while True:
    #     com.send_packet(Bytes.Address.ACTUATOR_1, Bytes.Command.STATUS)
    #     start = time.time()
    #     while not com.available():
    #         parse_serial()
    #         if time.time() - start > TIMEOUT:
    #             return False
    #     cmd = com.read()
    #     if cmd.cmd == bytes([Bytes.Command.STATUS]):
    #         print(cmd.payload[0])
    #         if cmd.payload[0] == Bytes.Status.IDLE:
    #             return True
    
print(pos())
wps = [Waypoint(0, 0, 0), Waypoint(3.14/2, 0, 5000), Waypoint(0, 0, 10000)]
traj = ActuatorTrajectory(wps)
print(load_traj(traj))
print(exec_traj())  

import serial
from Trajectory import *
from SerialParser import *
from Serialization import *

PORT = '/dev/ttyUSB0'
BAUD = 115200

PING = 0x01
HOME = 0x02
POS  = 0x03
TRAJ = 0x04
ACK  = 0xEE
NACK = 0xFF

ser = serial.Serial(PORT, BAUD, timeout=0.1)

cmds = []
def callback(cmd, payload):
    cmds.append((cmd, payload))

def get_pos():
    com.send_packet(POS)
    while len(cmds) == 0:
        com.read_input()
    print(read_float_le(cmds[0][1]))

com = SerialProtocol(ser, callback)

get_pos()
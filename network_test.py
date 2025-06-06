import socket
import time
from Trajectory import *
from SerialProtocol import *
from Serialization import *

# Configuration for TCP connection
ESP_IP = '192.168.20.117'  # Replace with your ESP32/ESP8266 IP address
ESP_PORT = 8000         # Replace with the port your ESP server listens on
TIMEOUT = 2          # Timeout in seconds

# Create and connect the TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(1)  # Non-blocking recv with short timeout
sock.connect((ESP_IP, ESP_PORT))

# Write callback for SerialProtocol
def write_callback(packet):
    sock.sendall(packet)

# Initialize protocol
com = SerialProtocol(Bytes.Address.MASTER, write_callback)

# Handle incoming TCP data
def parse_tcp():
    try:
        data = sock.recv(1024)
        for byte in data:
            com.feed(byte)
    except socket.timeout:
        pass
    except BlockingIOError:
        pass

# Commands adapted to TCP

def pos():
    com.send_packet(Bytes.Address.MASTER, Bytes.Command.POS)
    start = time.time()
    while True:
        parse_tcp()
        if time.time() - start > TIMEOUT:
            return False
        if com.available():
            cmd = com.read()
            if cmd.cmd == bytes([Bytes.Command.POS]):
                return read_float_le(cmd.payload)

def ping():
    com.send_packet(Bytes.Address.MASTER, Bytes.Command.PING)
    start = time.time()
    while time.time() - start < TIMEOUT:
        parse_tcp()
        if com.available():
            cmd = com.read()
            return cmd.cmd == bytes([Bytes.Command.ACK])
    return False

def estop():
    com.send_packet(Bytes.Address.MASTER, Bytes.Command.ESTOP)
    start = time.time()
    while time.time() - start < TIMEOUT:
        parse_tcp()
        if com.available():
            cmd = com.read()
            return cmd.cmd == bytes([Bytes.Command.ACK])
    return False

def home():
    com.send_packet(Bytes.Address.MASTER, Bytes.Command.HOME)
    start = time.time()
    while time.time() - start < TIMEOUT:
        parse_tcp()
        if com.available():
            cmd = com.read()
            if cmd.cmd != bytes([Bytes.Command.ACK]):
                print("ACK not received")
                return False
            print("ACKED")
            break
    else:
        return False

    while True:
        com.send_packet(Bytes.Address.MASTER, Bytes.Command.STATUS)
        start = time.time()
        while time.time() - start < TIMEOUT:
            parse_tcp()
            if com.available():
                cmd = com.read()
                if cmd.cmd == bytes([Bytes.Command.STATUS]) and cmd.payload[0] == Bytes.Status.IDLE:
                    return True
        return False

def load_traj(trajectory: ActuatorTrajectory):
    com.send_packet(Bytes.Address.MASTER, Bytes.Command.LOAD_TRAJ, trajectory.serialize())
    start = time.time()
    while time.time() - start < TIMEOUT:
        parse_tcp()
        if com.available():
            cmd = com.read()
            return cmd.cmd == bytes([Bytes.Command.ACK])
    return False

def exec_traj():
    com.send_packet(Bytes.Address.MASTER, Bytes.Command.EXEC_TRAJ)
    start = time.time()
    while time.time() - start < TIMEOUT:
        parse_tcp()
        if com.available():
            cmd = com.read()
            if cmd.cmd != bytes([Bytes.Command.ACK]):
                print("ACK not received")
                return False
            return True
    return False

# Example usage
if __name__ == '__main__':
    print("Position:", ping())
    print("Position:", pos())
    wps = [Waypoint(0, 0, 0), Waypoint(3.14/2, 0, 5000), Waypoint(0, 0, 10000)]
    traj = ActuatorTrajectory(wps)
    print("Loaded:", load_traj(traj))
    print("Executed:", exec_traj())
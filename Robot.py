import socket
import time
from Trajectory import *
from SerialProtocol import *
from Serialization import *

class ProtocolTimeoutError(BaseException):
    pass

class NackError(BaseException):
    pass

class Robot:
    def __init__(self, ip, port, tcp_timeout=1, protocol_timeout=1):
        self.ip = ip
        self.port = port
        self.tcp_timeout = tcp_timeout
        self.protocol_timeout = protocol_timeout

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.tcp_timeout)
        self.sock.connect((self.ip, self.port))
        self.master_com = SerialProtocol(Bytes.Address.MASTER, self._write_callback)

    def _write_callback(self, packet):
        self.sock.sendall(packet)

    def _parse_tcp(self):
        try:
            data = self.sock.recv(1024)
            for byte in data:
                self.master_com.feed(byte)
        except socket.timeout:
            pass
        except BlockingIOError:
            pass

    def _wait_for_packet(self, cmd: int):
        start = time.time()
        while time.time() - start < self.protocol_timeout:
            self._parse_tcp()
            if self.master_com.available():
                packet = self.master_com.read()
                if packet.cmd == bytes([cmd]):
                    return packet
                elif packet.cmd == bytes([Bytes.Command.NACK]):
                    raise NackError
        raise ProtocolTimeoutError

    def pos(self):
        self.master_com.send_packet(Bytes.Address.MASTER, Bytes.Command.POS)
        packet = self._wait_for_packet(Bytes.Command.POS)
        return RobotPosition.from_bytes(packet.payload)
    
    def ping(self):
        self.master_com.send_packet(Bytes.Address.MASTER, Bytes.Command.PING)
        self._wait_for_packet(Bytes.Command.ACK)
        return True
    
    def estop(self):
        self.master_com.send_packet(Bytes.Address.MASTER, Bytes.Command.ESTOP)
        self._wait_for_packet(Bytes.Command.ACK)
        return True
    
    def load_traj(self, trajectory: ActuatorTrajectory):
        self.master_com.send_packet(Bytes.Address.MASTER, Bytes.Command.LOAD_TRAJ, trajectory.serialize())
        self._wait_for_packet(Bytes.Command.ACK)
        return True
    
    def exec_traj(self):
        self.master_com.send_packet(Bytes.Address.MASTER, Bytes.Command.EXEC_TRAJ)
        self._wait_for_packet(Bytes.Command.ACK)
        return True
import socket
import time
from Trajectory import *
from SerialProtocol import *
from Serialization import *

class RobotControllerTimeout(BaseException):
    pass

class NackResponse(BaseException):
    pass

class Robot:
    def __init__(self, ip, port, tcp_timeout=1, protocol_timeout=1):
        self.ip = ip
        self.port = port
        self.tcp_timeout = tcp_timeout
        self.protocol_timeout = protocol_timeout

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.tcp_timeout)
        self._sock.connect((self.ip, self.port))
        self._master_com = SerialProtocol(Bytes.Address.MASTER, self._write_callback)

    def _write_callback(self, packet) -> None:
        self._sock.sendall(packet)

    def _parse_tcp(self) -> None:
        try:
            data = self._sock.recv(1024)
            for byte in data:
                self._master_com.feed(byte)
        except socket.timeout:
            pass
        except BlockingIOError:
            pass

    def _wait_for_packet(self, cmd: int) -> Command:
        start = time.time()
        while time.time() - start < self.protocol_timeout:
            self._parse_tcp()
            if self._master_com.available():
                packet = self._master_com.read()
                if packet.cmd == bytes([cmd]):
                    return packet
                elif packet.cmd == bytes([Bytes.Command.NACK]):
                    raise NackResponse
        raise RobotControllerTimeout
    
    def _wait_for_ack(self) -> None:
        self._wait_for_packet(Bytes.Command.ACK)

    def pos(self) -> RobotPosition:
        self._master_com.send_packet(Bytes.Command.POS)
        packet = self._wait_for_packet(Bytes.Command.POS)
        return RobotPosition.from_bytes(packet.payload)
    
    def ping(self) -> None:
        self._master_com.send_packet(Bytes.Command.PING)
        self._wait_for_ack()
    
    def estop(self) -> None:
        self._master_com.send_packet(Bytes.Command.ESTOP)
        self._wait_for_ack()
    
    def load_traj(self, trajectory: ActuatorTrajectory) -> None:
        self._master_com.send_packet(Bytes.Command.LOAD_TRAJ, trajectory.serialize())
        self._wait_for_ack()

    def exec_traj(self) -> None:
        self._master_com.send_packet(Bytes.Command.EXEC_TRAJ)
        self._wait_for_ack()
from enum import Enum, auto
import struct
from collections import deque

START_BYTE = 0xAA
ESCAPE_BYTE = 0xAB
ESCAPE_MASK = 0x20

PAYLOAD_BUFFER_SIZE = 1024
MAX_PACKET_BUFFER_SIZE = PAYLOAD_BUFFER_SIZE + 4
CMD_QUEUE_SIZE = 8

class ParserState(Enum):
    WAIT_START = auto()
    READ_ADDR = auto()
    READ_CMD = auto()
    READ_LEN = auto()
    READ_PAYLOAD = auto()
    READ_CHECKSUM = auto()

class Command:
    def __init__(self, cmd: bytes, payload: bytes):
        self.cmd = cmd
        self.payload = payload

class SerialParser:
    def __init__(self, address):
        self._state = ParserState.WAIT_START
        self._payload = bytearray()
        self._length = bytearray(2)
        self._length_bytes_read = 0
        self._cmd = None
        self._checksum = None
        self._crc_acc = 0x00
        self._escape_next = False
        self._address = address

        self._queue = deque(maxlen=CMD_QUEUE_SIZE)

    def parse(self, byte):
        if byte == START_BYTE:
            self._reset()
            self._state = ParserState.READ_CMD
            return
        
        if self._state != ParserState.WAIT_START:
            if self._escape_next:
                byte ^= ESCAPE_MASK
                self._escape_next = False
            elif byte == ESCAPE_BYTE:
                self._escape_next = True
                return
            
        match self._state:
            case ParserState.READ_ADDR:
                self._crc8_acc(byte)
                if (byte == self._address):
                    self._state = ParserState.READ_CMD
                else:
                    self._reset()

            case ParserState.READ_CMD:
                self._cmd = bytes([byte])
                self._crc8_acc(byte)
                self._state = ParserState.READ_LEN
                
            case ParserState.READ_LEN:
                self._length[self._length_bytes_read] = byte
                self._crc8_acc(byte)
                self._length_bytes_read += 1

                if self._length_bytes_read == 2:
                    length_val = struct.unpack('<H', self._length)[0]
                    if 0 <= length_val <= PAYLOAD_BUFFER_SIZE:
                        self._length_bytes_read = 0
                        if length_val == 0:
                            self._state = ParserState.READ_CHECKSUM
                        else:
                            self._payload = bytearray()
                            self._expected_length = length_val
                            self._state = ParserState.READ_PAYLOAD
                    else:
                        print("Payload too large")
                        self._reset()

            case ParserState.READ_PAYLOAD:
                self._crc8_acc(byte)
                self._payload.append(byte)
                if len(self._payload) >= struct.unpack('<H', self._length)[0]:
                    self._state = ParserState.READ_CHECKSUM

            case ParserState.READ_CHECKSUM:
                self._checksum = byte
                self._state = ParserState.WAIT_START
                self._validate()

    def available(self) -> int:
        return len(self._queue)

    def read(self):
        if not self._queue:
            return None
        return self._queue.popleft()
    
    def _reset(self):
        self._state = ParserState.WAIT_START
        self._payload = bytearray()
        self._length = bytearray(2)
        self._length_bytes_read = 0
        self._cmd = None
        self._checksum = None
        self._crc_acc = 0x00
        self._escape_next = False

    def _validate(self):
        if self._crc_acc == self._checksum:
            self._enqueue_command(self._cmd, bytes(self._payload))
        else:
            print("Checksum Failed!")

    def _enqueue_command(self, cmd: bytes, payload: bytes):
        if len(self._queue) < CMD_QUEUE_SIZE:
            self._queue.append(Command(cmd, payload))
        else:
            print("Command queue full, dropping command")

    def _crc8_acc(self, byte: int):
        self._crc_acc ^= byte
        for _ in range(8):
            self._crc_acc = (self._crc_acc << 1) ^ 0x07 if (self._crc_acc & 0x80) else (self._crc_acc << 1)
            self._crc_acc &= 0xFF

class SerialProtocol:
    def __init__(self, serial, address):
        self._serial = serial
        self._parser = SerialParser(address)
        self._address = address

    def available(self):
        self._read_serial()
        return self._parser.available()
    
    def read(self):
        return self._parser.read()
    
    def send_packet(self, addr: int, cmd: int, payload: bytes = bytes([])):
        length = struct.pack('<H', len(payload))
        packet = bytes([addr]) + bytes([cmd]) + length + payload
        checksum = self._crc8(packet)
        packet = packet + bytes([checksum])
        packet = self._escape_packet(packet)
        packet = bytes([START_BYTE]) + packet
        self._serial.write(packet)

    def _read_serial(self):
        while self._serial.in_waiting > 0:
            byte = self._serial.read(1)
            if byte:
                self._parser.parse(byte[0])
                
    def _crc8(self, data: bytes) -> int:
        crc: int = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc << 1) ^ 0x07 if (crc & 0x80) else (crc << 1)
                crc &= 0xFF
        return crc

    def _escape_packet(self, data: bytes) -> bytes:
        escaped = bytearray()
        for b in data:
            if b == START_BYTE or b == ESCAPE_BYTE:
                escaped.append(ESCAPE_BYTE)
                escaped.append(b ^ ESCAPE_MASK)
            else:
                escaped.append(b)
        return bytes(escaped)

    

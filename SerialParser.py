from enum import Enum, auto
import struct

START_BYTE = 0xAA
ESCAPE_BYTE = 0xAB
ESCAPE_MASK = 0x20

PAYLOAD_BUFFER_SIZE = 1024
MAX_PACKET_BUFFER_SIZE = PAYLOAD_BUFFER_SIZE + 4

class ParserState(Enum):
    WAIT_START = auto()
    READ_CMD = auto()
    READ_LEN = auto()
    READ_PAYLOAD = auto()
    READ_CHECKSUM = auto()

class SerialParser:
    def __init__(self, dispatch_callback=None):
        self.dispatch_callback = dispatch_callback
        self.parser_state = ParserState.WAIT_START
        self.payload = []
        self.length = None
        self.length_bytes_read = 0
        self.cmd = None
        self.checksum = None
        self.crc_acc = 0x00
        self.escape_next = False

    def parse(self, byte):

        if byte == START_BYTE:
            self.reset_parser()
            self.parser_state = ParserState.READ_CMD
            return
        
        if (self.parser_state != ParserState.WAIT_START):
            if self.escape_next:
                byte ^= ESCAPE_MASK
                self.escape_next = False
            elif byte == ESCAPE_BYTE:
                self.escape_next = True
                return
            
        match self.parser_state:
            case ParserState.READ_CMD:
                self.cmd = byte
                self.crc8_acc(byte)
                self.parser_state = ParserState.READ_LEN
                
            case ParserState.READ_LEN:
                if self.length_bytes_read == 0:
                    self.length = byte
                    self.length_bytes_read = 1
                    self.crc8_acc(byte)

                else:
                    self.length |= (byte << 8)
                    self.crc8_acc(byte)
                    self.length_bytes_read = 0

                    if self.length > 0 and self.length <= PAYLOAD_BUFFER_SIZE:
                        self.parser_state = ParserState.READ_PAYLOAD
                    elif self.length == 0:
                        self.parser_state = ParserState.READ_CHECKSUM
                    else:
                        print("Payload too large")
                        self.reset_parser()

            case ParserState.READ_PAYLOAD:
                self.crc8_acc(byte)
                self.payload.append(byte)
                if len(self.payload) >= self.length:
                    self.parser_state = ParserState.READ_CHECKSUM

            case ParserState.READ_CHECKSUM:
                self.checksum = byte
                self.parser_state = ParserState.WAIT_START
                self.validate()

    def reset_parser(self):
        self.parser_state = ParserState.WAIT_START
        self.payload = []
        self.length = 0
        self.crc_acc = 0x00
        self.escape_next = False

    def validate(self):
        if self.crc_acc == self.checksum:
            self.dispatch()
        else:
            print("Checksum Failed!")

    def dispatch(self):
        if self.dispatch_callback:
            self.dispatch_callback(self.cmd, self.payload)
        
    def crc8_acc(self, byte: int):
        self.crc_acc ^= byte
        for _ in range(8):
            self.crc_acc = (self.crc_acc << 1) ^ 0x07 if (self.crc_acc & 0x80) else (self.crc_acc << 1)
            self.crc_acc &= 0xFF

class SerialProtocol:
    def __init__(self, serial, dispatch_callback=None):
        self.serial = serial
        self.parser = SerialParser(dispatch_callback)

    def crc8(self, data: bytes) -> int:
        crc: int = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc << 1) ^ 0x07 if (crc & 0x80) else (crc << 1)
                crc &= 0xFF
        return crc

    def escape_packet(self, data: bytes) -> bytes:
        escaped = bytearray()
        for b in data:
            if b == START_BYTE or b == ESCAPE_BYTE:
                escaped.append(ESCAPE_BYTE)
                escaped.append(b ^ ESCAPE_MASK)
            else:
                escaped.append(b)
        return bytes(escaped)

    def send_packet(self, cmd: int, payload: bytes):
        length = struct.pack('<H', len(payload))
        packet = bytes([cmd]) + length + payload
        checksum = self.crc8(packet)
        packet = packet + bytes([checksum])
        packet = self.escape_packet(packet)
        packet = bytes([START_BYTE]) + packet
        self.serial.write(packet)

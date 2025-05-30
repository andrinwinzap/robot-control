import struct

def read_uint16_le(buffer: bytes) -> int:
    return struct.unpack('<H', buffer[:2])[0]

def read_uint32_le(buffer: bytes) -> int:
    return struct.unpack('<I', buffer[:4])[0]

def read_float_le(buffer: bytes) -> float:
    return struct.unpack('<f', buffer[:4])[0]

def write_uint16_le(value: int) -> bytes:
    return struct.pack('<H', value)

def write_uint32_le(value: int) -> bytes:
    return struct.pack('<I', value)

def write_float_le(value: float) -> bytes:
    return struct.pack('<f', value)

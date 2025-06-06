import struct
from typing import List

MAX_WAYPOINTS = 64

class Waypoint:
    def __init__(self, position: float, velocity: float, timestamp: int):
        self.position = position
        self.velocity = velocity
        self.timestamp = timestamp

    def __repr__(self):
        return f"Waypoint(position={self.position}, velocity={self.velocity}, timestamp={self.timestamp})"

class ActuatorTrajectory:
    def __init__(self, waypoints: List[Waypoint] = []):
        self.waypoints = waypoints

    @staticmethod
    def write_float_le(value: float) -> bytes:
        return struct.pack('<f', value)
        
    @staticmethod
    def write_uint32_le(value: int) -> bytes:
        return struct.pack('<I', value)

    def serialize(self) -> bytes:
        total_bytes = 1 + len(self.waypoints) * 12
        out_buffer = bytearray(total_bytes)
        out_buffer[0] = len(self.waypoints)
        idx = 1
        for wp in self.waypoints:
            out_buffer[idx:idx+4] = self.write_float_le(wp.position)
            idx += 4
            out_buffer[idx:idx+4] = self.write_float_le(wp.velocity)
            idx += 4
            out_buffer[idx:idx+4] = self.write_uint32_le(wp.timestamp)
            idx += 4
        return bytes(out_buffer)
    
    def __repr__(self):
        return f"Trajectory(length={len(self.waypoints)}, waypoints={self.waypoints})"

if __name__ == "__main__":
    wps = [Waypoint(1,2,3),Waypoint(1,3,3), Waypoint(1,2,3), Waypoint(1,2,3), Waypoint(1,2,3), Waypoint(1,2,3)]
    traj = ActuatorTrajectory(wps)
    print(traj.serialize())
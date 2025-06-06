import struct
from typing import List
from Serialization import *

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

    def serialize(self) -> bytes:
        total_bytes = 1 + len(self.waypoints) * 12
        out_buffer = bytearray(total_bytes)
        out_buffer[0] = len(self.waypoints)
        idx = 1
        for wp in self.waypoints:
            out_buffer[idx:idx+4] = write_float_le(wp.position)
            idx += 4
            out_buffer[idx:idx+4] = write_float_le(wp.velocity)
            idx += 4
            out_buffer[idx:idx+4] = write_uint32_le(wp.timestamp)
            idx += 4
        return bytes(out_buffer)
    
    def __repr__(self):
        return f"Trajectory(length={len(self.waypoints)}, waypoints={self.waypoints})"
    
class RobotTrajectory:
    def __init__(self,
                 actuator_1: ActuatorTrajectory,
                 actuator_2: ActuatorTrajectory, 
                 actuator_3: ActuatorTrajectory, 
                 actuator_4: ActuatorTrajectory, 
                 actuator_5: ActuatorTrajectory, 
                 actuator_6: ActuatorTrajectory
                 ):
        self.actuator_1 = actuator_1
        self.actuator_2 = actuator_2
        self.actuator_3 = actuator_3
        self.actuator_4 = actuator_4
        self.actuator_5 = actuator_5
        self.actuator_6 = actuator_6

    def serialize(self) -> bytes:
        # Serialize each actuator and concatenate
        return (
            self.actuator_1.serialize() +
            self.actuator_2.serialize() +
            self.actuator_3.serialize() +
            self.actuator_4.serialize() +
            self.actuator_5.serialize() +
            self.actuator_6.serialize()
        )

    def __repr__(self):
        return (
            f"RobotTrajectory(\n"
            f"  actuator_1={self.actuator_1},\n"
            f"  actuator_2={self.actuator_2},\n"
            f"  actuator_3={self.actuator_3},\n"
            f"  actuator_4={self.actuator_4},\n"
            f"  actuator_5={self.actuator_5},\n"
            f"  actuator_6={self.actuator_6}\n"
            f")"
        )

if __name__ == "__main__":
    wps = [Waypoint(1,2,3),Waypoint(1,3,3), Waypoint(1,2,3), Waypoint(1,2,3), Waypoint(1,2,3), Waypoint(1,2,3)]
    traj = ActuatorTrajectory(wps)
    print(traj.serialize())
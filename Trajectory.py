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

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) < 1:
            return cls([])

        count = data[0]
        if count > MAX_WAYPOINTS or len(data) < 1 + count * 12:
            return cls([])

        waypoints = []
        idx = 1
        for _ in range(count):
            position = read_float_le(data[idx:idx+4])
            idx += 4
            velocity = read_float_le(data[idx:idx+4])
            idx += 4
            timestamp = read_uint32_le(data[idx:idx+4])
            idx += 4
            waypoints.append(Waypoint(position, velocity, timestamp))
        return cls(waypoints)

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
                 actuator_1: ActuatorTrajectory = ActuatorTrajectory(),
                 actuator_2: ActuatorTrajectory = ActuatorTrajectory(),
                 actuator_3: ActuatorTrajectory = ActuatorTrajectory(),
                 actuator_4: ActuatorTrajectory = ActuatorTrajectory(),
                 actuator_5: ActuatorTrajectory = ActuatorTrajectory(),
                 actuator_6: ActuatorTrajectory = ActuatorTrajectory()
                 ):
        self.actuator_1 = actuator_1
        self.actuator_2 = actuator_2
        self.actuator_3 = actuator_3
        self.actuator_4 = actuator_4
        self.actuator_5 = actuator_5
        self.actuator_6 = actuator_6

    @classmethod
    def from_bytes(cls, data: bytes):
        idx = 0

        a1 = ActuatorTrajectory.from_bytes(data[idx:])
        bytes_a1 = 1 + len(a1.waypoints) * 12
        idx += bytes_a1

        a2 = ActuatorTrajectory.from_bytes(data[idx:])
        bytes_a2 = 1 + len(a2.waypoints) * 12
        idx += bytes_a2

        a3 = ActuatorTrajectory.from_bytes(data[idx:])
        bytes_a3 = 1 + len(a3.waypoints) * 12
        idx += bytes_a3

        a4 = ActuatorTrajectory.from_bytes(data[idx:])
        bytes_a4 = 1 + len(a4.waypoints) * 12
        idx += bytes_a4

        a5 = ActuatorTrajectory.from_bytes(data[idx:])
        bytes_a5 = 1 + len(a5.waypoints) * 12
        idx += bytes_a5

        a6 = ActuatorTrajectory.from_bytes(data[idx:])
        # no need to increment idx after last one

        return cls(a1, a2, a3, a4, a5, a6)

    def serialize(self) -> bytes:
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

class RobotPosition:
    def __init__(self, theta_1=0.0, theta_2=0.0, theta_3=0.0, theta_4=0.0, theta_5=0.0, theta_6=0.0):
        self.theta_1 = theta_1
        self.theta_2 = theta_2
        self.theta_3 = theta_3
        self.theta_4 = theta_4
        self.theta_5 = theta_5
        self.theta_6 = theta_6

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) < 24:
            raise ValueError("Insufficient bytes for RobotPosition")
        theta_1 = read_float_le(data[0:4])
        theta_2 = read_float_le(data[4:8])
        theta_3 = read_float_le(data[8:12])
        theta_4 = read_float_le(data[12:16])
        theta_5 = read_float_le(data[16:20])
        theta_6 = read_float_le(data[20:24])
        return cls(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)

    def serialize(self) -> bytes:
        out = bytearray(24)
        out[0:4] = write_float_le(self.theta_1)
        out[4:8] = write_float_le(self.theta_2)
        out[8:12] = write_float_le(self.theta_3)
        out[12:16] = write_float_le(self.theta_4)
        out[16:20] = write_float_le(self.theta_5)
        out[20:24] = write_float_le(self.theta_6)
        return bytes(out)

    def __repr__(self):
        return (f"RobotPosition(theta_1={self.theta_1}, theta_2={self.theta_2}, "
                f"theta_3={self.theta_3}, theta_4={self.theta_4}, "
                f"theta_5={self.theta_5}, theta_6={self.theta_6})")

if __name__ == "__main__":
    wps = [
        Waypoint(1, 2, 3),
        Waypoint(1, 3, 3),
        Waypoint(1, 2, 3),
        Waypoint(1, 2, 3),
        Waypoint(1, 2, 3),
        Waypoint(1, 2, 3)
    ]
    traj = ActuatorTrajectory(wps)
    print(traj.serialize())

    # Example RobotPosition usage
    pos = RobotPosition(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    serialized_pos = pos.serialize()
    print(serialized_pos)
    deserialized_pos = RobotPosition.from_bytes(serialized_pos)
    print(deserialized_pos)

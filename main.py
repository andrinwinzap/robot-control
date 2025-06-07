from Robot import *

robot = Robot('192.168.20.117', 8000)

trajectory = ActuatorTrajectory([
                                Waypoint(0, 0, 0), 
                                Waypoint(3.14/2, 0, 5000), 
                                Waypoint(0, 0, 10000)
                                ])

print("Ping:", robot.ping())
print("Position:", robot.pos())
print("Trajectory loaded:", robot.load_traj(trajectory))
print("Trajectory executed:", robot.exec_traj())
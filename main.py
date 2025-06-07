from Robot import *

robot = Robot('192.168.20.117', 8000)

actuator_1_trajectory = ActuatorTrajectory([
                                Waypoint(0, 0, 0), 
                                Waypoint(3.14/2, 0, 5000), 
                                Waypoint(0, 0, 10000)
                                ])

actuator_2_trajectory = ActuatorTrajectory([
                                Waypoint(0, 0, 0), 
                                Waypoint(3.14/2, 0, 5000), 
                                Waypoint(0, 0, 10000)
                                ])

actuator_3_trajectory = ActuatorTrajectory([
                                Waypoint(0, 0, 0), 
                                Waypoint(3.14/2, 0, 5000), 
                                Waypoint(0, 0, 10000)
                                ])

actuator_4_trajectory = ActuatorTrajectory([
                                Waypoint(0, 0, 0), 
                                Waypoint(3.14/2, 0, 5000), 
                                Waypoint(0, 0, 10000)
                                ])

trajectory = RobotTrajectory(actuator_1_trajectory,actuator_2_trajectory,actuator_3_trajectory,actuator_4_trajectory)

robot.ping()
position = robot.pos()
print("Position:", position)
robot.load_traj(trajectory)
robot.exec_traj()
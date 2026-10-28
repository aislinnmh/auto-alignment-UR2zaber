## Auto-alignment of the UR5e robot with the Zaber rotation stage yaw rotation using pitag tracking
# Aislinn Hurley 10/22/2026

###################################################################################

import numpy as np
import time
import matplotlib

from align_axes import correct_eye_position
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Set up robot communication
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

robot_ip = "192.168.1.39"
rtde_c = RTDEControl(robot_ip)
rtde_r = RTDEReceive(robot_ip)

# Set up zaber stage communication
from zaber_motion.ascii import Connection;
from zaber_motion import Units
connection = Connection.open_serial_port('COM8')
connection.enable_alerts()
deviceList = connection.detect_devices()
device1 = deviceList[0]
yawRot = device1.get_axis(1)

# Set up pupil tracking communication
from position_tracking import get_pos 
mode = "pitag"

# Check initial positions
tcp_pose = rtde_r.getActualTCPPose()
print("Initial TCP pose:", tcp_pose)
# x = xAxis.get_position(Units.LENGTH_MILLIMETRES)
# y = yAxis.get_position(Units.LENGTH_MILLIMETRES)
# z = zAxis.get_position(Units.LENGTH_MILLIMETRES)

print("1) Start pitag tracking in a Terminal with -a localhost")
print("2) Manually move the zaber stages such that the eye is within 2 mm of the center")
input("Press Enter to continue when complete...")

# Set parameters
max_iter = 50
tol = 0.01 # mm

# def correct_eye_position():
#     # Moves eye to the center of the pitag tracking system with zaber stages
#     dtol = 0.02;
#     pos = get_pos(mode)
#     d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
#     while d > dtol:
#         xAxis.move_relative(pos[0], Units.LENGTH_MILLIMETRES)
#         yAxis.move_relative(pos[1], Units.LENGTH_MILLIMETRES)
#         zAxis.move_relative(pos[2], Units.LENGTH_MILLIMETRES)
#         time.sleep(1)
#         pos = get_pos(mode)
#         d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)

# correct_eye_position()

iteration_history = []

# for i in range(max_iter):

#     print(f"Iteration {i+1} of {max_iter}")

    ## YAW
   
    # ## Pitch rotation
    # # Get current relative rotations
    # tcp_pose = rtde_r.getActualTCPPose()
    # ry = tcp_pose[4]
    # # Move to start position, +10 deg
    # xAxis.move_relative(10.0, Units.ANGLE_DEGREES) ## MOVE YAW, measure pitch and roll change 
    # # Record pupil position 
    # time.sleep(1)
    # pos = get_pos(mode)
    # pitch10 = pos[4]
    # print(f"Current pitch position: {pitch10:.2f}")  
    # # y5 = float(input("Enter current y position: ")) 
    # # Jump position to -10 deg
    # xAxis.move_relative(-20.0, Units.ANGLE_DEGREES) ## MOVE YAW, measure pitch and roll change 
    # # Record pupil position
    # time.sleep(1)
    # pos = get_pos(mode)
    # pitch_10 = pos[4]
    # print(f"Current pitch position: {pitch_10:.2f}")
    # # y_5 = float(input("Enter current y position: "))
    # # Move back to center
    # xAxis.move_relative(10.0, Units.ANGLE_DEGREES)
    # # Calculate differences in pupil position and the corresponding rotation adjustments
    # dpitch = pitch_10 - pitch10
    # if abs(dpitch) > tol:
    #     # d_theta = dpitch/0 # radians, small angle approx.
    #     # d_theta = d_theta*0.75
    #     # ry = ry - d_theta/

for i in range(max_iter):
    print(f"Iteration {i+1} of {max_iter}")

    rxs = [-1,0,1]
    rys = [-1,0,1]
    rxs = np.array(rxs)/(i+1) 
    rys = np.array(rys)/(i+1)
    rxs = rxs/180*np.pi
    rys = rys/180*np.pi
    rxy_list = []
    d_list = []

    tcp_pose = rtde_r.getActualTCPPose()

    for rx in rxs:
        for ry in rys:

            rxy_list.append((rx, ry))

            rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+rx, tcp_pose[4]+ry, tcp_pose[5]], 0.125, 0.125, True)
            time.sleep(1)

            print(f"Testing rx: {rx}, ry: {ry}")
            yawRot.move_relative(10.0, Units.ANGLE_DEGREES)
            time.sleep(1)
            pos = get_pos(mode)
            pitch10 = pos[4]
            roll10 = pos[5]
            yawRot.move_relative(-20.0, Units.ANGLE_DEGREES)
            time.sleep(1)
            pos = get_pos(mode)
            pitch_10 = pos[4]
            roll_10 = pos[5]
            yawRot.move_relative(10.0, Units.ANGLE_DEGREES)
            dpitch = pitch_10 - pitch10
            droll = roll_10 - roll10
            d_avg = (dpitch + droll) / 2
            print(f"dpitch: {dpitch:.2f}, droll: {droll:.2f}, d_avg: {d_avg:.2f}")

            d_list.append(d_avg)

    print(f"d_list: {d_list}")
    print(f"rxy_list: {rxy_list}")
    best_idx = int(np.argmin(d_list))
    best_rx, best_ry = rxy_list[best_idx]
    best_d = d_list[best_idx]

    print(f"Minimum d_avg: {best_d:.2f} at rx={best_rx}, ry={best_ry}")

    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+best_rx, tcp_pose[4]+best_ry, tcp_pose[5]], 0.125, 0.125, True)
    time.sleep(1)



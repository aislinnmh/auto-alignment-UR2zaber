## Auto-alignment of the UR5e robot with the Zaber rotation stage yaw rotation using pitag tracking
# Aislinn Hurley 10/22/2026

###################################################################################

import numpy as np
import time
import matplotlib

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
print("2) Manually move the robot such that the eye is within 1 degree of the center")
input("Press Enter to continue when complete...")

# Set parameters
max_iter = 10
# tol = 0.01 # mm

def correct_eye_position():
    # Moves eye to the center of the pitag tracking system with zaber stages
    dtol = 0.5
    pos = get_pos(mode)
    if pos is not None:
        d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
        step = 0.25/1000
        tcp_pose = rtde_r.getActualTCPPose()
        while d > dtol:
            tcp_pose = rtde_r.getActualTCPPose()
            x = pos[0]/1000
            y = pos[1]/1000
            z = pos[2]/1000
            sign_x = np.sign(x)
            sign_y = np.sign(y)
            sign_z = np.sign(z)
            rtde_c.moveL([tcp_pose[0]+(sign_x*step), tcp_pose[1]-(sign_z*step), tcp_pose[2]+(sign_y*step), tcp_pose[3], tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
            time.sleep(0.25)
            pos = get_pos(mode)
            d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            # print(d)
    else:
        print("Manually fix robot position.")
        input("Press Enter to continue when complete...")
        d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
        step = 0.25/1000
        tcp_pose = rtde_r.getActualTCPPose()
        while d > dtol:
            tcp_pose = rtde_r.getActualTCPPose()
            x = pos[0]/1000
            y = pos[1]/1000
            z = pos[2]/1000
            sign_x = np.sign(x)
            sign_y = np.sign(y)
            sign_z = np.sign(z)
            rtde_c.moveL([tcp_pose[0]+(sign_x*step), tcp_pose[1]-(sign_z*step), tcp_pose[2]+(sign_y*step), tcp_pose[3], tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
            time.sleep(0.25)
            pos = get_pos(mode)
            d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)

def center_initial_rot():
    tcp_pose = rtde_r.getActualTCPPose()
    step = 0.1/180*np.pi
    atol = 0.5
    pos = get_pos(mode)
    pitch = pos[3]*180/np.pi
    yaw = pos[4]*180/np.pi
    roll = -pos[5]*180/np.pi
    a = np.abs(pitch) + np.abs(yaw) + np.abs(roll)
    while ((a > atol) or (np.abs(roll) > 1.5)) or pos is None:
        tcp_pose = rtde_r.getActualTCPPose()
        pos = get_pos(mode)
        if pos is None:
            print("Manually fix robot position.")
            input("Press Enter to continue when complete...")
        p_sign = -np.sign(pitch)
        y_sign = -np.sign(yaw)
        r_sign = -np.sign(roll)
        rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+p_sign*step, tcp_pose[4]+y_sign*step+r_sign*step, tcp_pose[5]+y_sign*step-r_sign*step], 0.125, 0.125, True)
        time.sleep(0.5)
        correct_eye_position()

        pos = get_pos(mode)
        if pos is not None:
            pitch = pos[3]*180/np.pi
            yaw = pos[4]*180/np.pi
            roll = -pos[5]*180/np.pi
            a = np.abs(pitch) + np.abs(yaw) 

correct_eye_position()                                                            
center_initial_rot()
correct_eye_position()


iteration_history = []

total_rx = 0
total_ry = 0

# for i in range(max_iter):
#     print(f"Iteration {i+1} of {max_iter}")

#     # rxs = [-1,0,1]
#     rys = [-1,0,1]
#     rxs = [0]
#     rxs = np.array(rxs)/(i+1) 
#     rys = np.array(rys)/(i+1)
#     rxs = rxs/180*np.pi
#     rys = rys/180*np.pi
#     rxy_list = []
#     d_list = []

#     tcp_pose = rtde_r.getActualTCPPose()

#     # pos 3 4 5 --> pitch, yaw, roll in robot frame

#     for rx in rxs:
#         for ry in rys:

#             rxy_list.append((rx, ry))

#             rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+rx, tcp_pose[4]+ry, tcp_pose[5]+ry], 0.125, 0.125, True)
#             time.sleep(1)
#             correct_eye_position()

#             # print(f"Testing rx: {rx}, ry: {ry}")
#             yawRot.move_relative(10.0, Units.ANGLE_DEGREES)
#             time.sleep(1)
#             pos = get_pos(mode)
#             pitch10 = pos[3]*180/np.pi
#             roll10 = pos[5]*180/np.pi
#             yawRot.move_relative(-20.0, Units.ANGLE_DEGREES)
#             time.sleep(1)
#             pos = get_pos(mode)
#             pitch_10 = pos[3]*180/np.pi
#             roll_10 = pos[5]*180/np.pi
#             yawRot.move_relative(10.0, Units.ANGLE_DEGREES)
#             dpitch = pitch_10 - pitch10
#             droll = roll_10 - roll10
#             d_avg = np.abs(dpitch)
#             # d_avg = (np.abs(dpitch) + np.abs(droll)) / 2
#             # print(f"dpitch: {dpitch:.2f}, droll: {droll:.2f}, d_avg: {d_avg:.2f}")

#             d_list.append(d_avg)

#     print(f"d_list: {d_list}")
#     print(f"rxy_list: {rxy_list}")
#     best_idx = int(np.argmin(d_list))
#     best_rx, best_ry = rxy_list[best_idx]
#     best_d = d_list[best_idx]
#     total_rx += best_rx
#     total_ry += best_ry

#     print(f"Minimum d_avg: {best_d:.2f} at rx={best_rx}, ry={best_ry}")

#     rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+best_rx, tcp_pose[4]+best_ry, tcp_pose[5]], 0.125, 0.125, True)
#     time.sleep(1)

#     print(f"Total rx: {total_rx}, Total ry: {total_ry}")

# correct_eye_position()
step = 0./180*np.pi
for i in range(max_iter):
    print(f"Iteration {i+1} of {max_iter}")

    tcp_pose = rtde_r.getActualTCPPose()

    # pos 3 4 5 --> pitch, yaw, roll in robot frame

    # print(f"Testing rx: {rx}, ry: {ry}")
    yawRot.move_relative(10.0, Units.ANGLE_DEGREES)
    time.sleep(1)
    pos = get_pos(mode)
    pitch10 = pos[3]*180/np.pi
    roll10 = pos[5]*180/np.pi
    yawRot.move_relative(-20.0, Units.ANGLE_DEGREES)
    time.sleep(1)
    pos = get_pos(mode)
    pitch_10 = pos[3]*180/np.pi
    roll_10 = pos[5]*180/np.pi
    yawRot.move_relative(10.0, Units.ANGLE_DEGREES)
    dpitch = pitch_10 - pitch10
    droll = roll_10 - roll10
    d_avg = (np.abs(dpitch) + np.abs(droll)) / 2
    print(f"dpitch: {dpitch:.2f}, droll: {droll:.2f}, d_avg: {d_avg:.2f}")

    rx = np.sign(dpitch) * step
    ry = np.sign(droll) * step

    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+rx, tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)

    # rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+rx, tcp_pose[4]+ry, tcp_pose[5]+ry], 0.125, 0.125, True)
    time.sleep(1)
    correct_eye_position()



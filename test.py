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

dr = 1/180*np.pi/np.sqrt(2)
# print('rx')
# rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3]+dr, tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
# time.sleep(1)
# rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
# time.sleep(2)
print('rz')
rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4]+dr, tcp_pose[5]+dr], 0.125, 0.125, True)
time.sleep(5)
input("Press Enter to continue...")
rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
time.sleep(2)
print('ry')
rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4]+dr, tcp_pose[5]-dr], 0.125, 0.125, True)
input("Press Enter to continue...")
time.sleep(5)
rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)

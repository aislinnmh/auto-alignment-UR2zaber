## Auto-alignment of the UR5e robot with the Zaber stage axes using pupil tracking
# Aislinn Hurley 10/22/2026

###################################################################################

import numpy as np

# Set up robot communication
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

robot_ip = "192.168.1.39"
rtde_c = RTDEControl(robot_ip)
rtde_r = RTDEReceive(robot_ip)

# Set up zaber stage communication
from zaber_motion.ascii import Connection;
from zaber_motion import Units
connection = Connection.open_serial_port('COM7')
connection.enable_alerts()
deviceList = connection.detect_devices()
device1 = deviceList[0]
device2 = deviceList[1]
device3 = deviceList[2]
xAxis = device1.get_axis(1)
zAxis = device2.get_axis(1) # Order flipped between y and z
yAxis = device3.get_axis(1)

# Set up pupil tracking communication
# INCOMPLETE

# Check initial positions
tcp_pose = rtde_r.getActualTCPPose()
print("Initial TCP pose:", tcp_pose)
# x = xAxis.get_position(Units.LENGTH_MILLIMETRES)
# y = yAxis.get_position(Units.LENGTH_MILLIMETRES)
# z = zAxis.get_position(Units.LENGTH_MILLIMETRES)

print("1) Start pupil tracking in a Terminal with -a localhost")
print("2) Manually move the zaber stages such that the eye is approximately centered at pupil_center_m")
input("Press Enter to continue when these two steps are complete...")

# Set parameters
max_iter = 10
tol = 0.01 # mm

for i in range(max_iter):

    ## X-AXIS ALIGNMENT
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    ry = tcp_pose[4]
    rz = tcp_pose[5]
    # Move to start position, +5 mm
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position TO BE REPLACED WITH PUPIL TRACKING DATA
    y5 = float(input("Enter current y position: ")) 
    z5 = float(input("Enter current z position: "))
    # Jump position to -5 mm
    xAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    y_5 = float(input("Enter current y position: "))
    z_5 = float(input("Enter current z position: "))
    # Move back to center
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dy = y_5 - y5
    dz = z_5 - z5
    if abs(dy) > tol:
        d_theta = dy/10 # radians, small angle approx.
        d_theta = d_theta *0.05
        ry = ry - d_theta
        dry1 = d_theta
    if abs(dz) > tol:
        d_theta = dz/10 # radians, small angle approx.
        d_theta = d_theta  *0.05
        rz = rz - d_theta
        drz1 = d_theta
    tcp_pose = rtde_r.getActualTCPPose()
    ry = tcp_pose[4]
    rz = tcp_pose[5]
    # Move to start position, +5 mm
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position TO BE REPLACED WITH PUPIL TRACKING DATA
    y5 = float(input("Enter current y position: ")) 
    z5 = float(input("Enter current z position: "))
    # Jump position to -5 mm
    xAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    y_5 = float(input("Enter current y position: "))
    z_5 = float(input("Enter current z position: "))
    # Move back to center
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dy = y_5 - y5
    dz = z_5 - z5
    if abs(dy) > tol:
        d_theta = dy/10 # radians, small angle approx.
        d_theta = d_theta *0.05
        ry = ry - d_theta
        dry1 = d_theta
    if abs(dz) > tol:
        d_theta = dz/10 # radians, small angle approx.
        d_theta = d_theta  *0.05
        rz = rz - d_theta
        drz1 = d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], ry, rz], 0.5, 0.5, True)
    # Correct eye position if needed TO BE REPLACED WITH PUPIL TRACKING DATA
    # input("Manually fix eye position if needed, then press Enter to continue...")

'''
    ## Y-AXIS ALIGNMENT
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    rx = tcp_pose[3]
    ry = tcp_pose[4]
    # Move to start position, +5 mm
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position TO BE REPLACED WITH PUPIL TRACKING DATA
    x5 = float(input("Enter current x position: "))
    z5 = float(input("Enter current z position: "))
    # Jump position to -5 mm
    yAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    x_5 = float(input("Enter current x position: "))
    z_5 = float(input("Enter current z position: "))
    # Move back to center
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position
    dx = x_5 - x5
    dz = z_5 - z5
    if abs(dx) > tol:
        d_theta = dx/10 # radians, small angle approx.
        ry = ry + d_theta
        dry2 = d_theta
    if abs(dz) > tol:
        d_theta = dz/10 # radians, small angle approx.
        rx = rx + d_theta
        drx1 = d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], rx, ry, tcp_pose[5]], 0.5, 0.5, True)
    # Correct eye position if needed TO BE REPLACED WITH PUPIL TRACKING DATA
    input("Manually fix eye position if needed, then press Enter to continue...") 
    
    ## Z-AXIS ALIGNMENT
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    rx = tcp_pose[3]
    rz = tcp_pose[5]
    # Move to start position, +5 mm
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position TO BE REPLACED WITH PUPIL TRACKING DATA
    x5 = float(input("Enter current x position: "))
    y5 = float(input("Enter current y position: "))
    # Jump position to -5 mm
    zAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    x_5 = float(input("Enter current x position: "))
    y_5 = float(input("Enter current y position: "))
    # Move back to center
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position
    dx = x_5 - x5
    dy = y_5 - y5
    if abs(dx) > tol:
        d_theta = dx/10 # radians, small angle approx.
        drz2 = d_theta
        rz = rz + d_theta
    if abs(dy) > tol:
        d_theta = dy/10 # radians, small angle approx.
        drx2 = d_theta
        rx = rx - d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], rx, tcp_pose[4], rz], 0.5, 0.5, True)
    # Correct eye position if needed TO BE REPLACED WITH PUPIL TRACKING DATA
    input("Manually fix eye position if needed, then press Enter to continue...")

    # # Adjust once per iteration
    # dry = (-dry1 + dry2) / 2
    # drx = (drx1 + drx2) / 2
    # drz = (-drz1 + drz2) / 2
    # rx = rx + drx
    # ry = ry + dry
    # rz = rz + drz
    # rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], rx, ry, rz], 0.5, 0.5, True)
    # input("Manually fix eye position if needed, then press Enter to continue...")
'''

rtde_c.disconnect()
rtde_r.disconnect()





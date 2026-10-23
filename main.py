## Auto-alignment of the UR5e robot with the Zaber stage axes using pupil tracking
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
from position_tracking import get_pos 
mode = "pupil" # or "pitag"

# Check initial positions
tcp_pose = rtde_r.getActualTCPPose()
print("Initial TCP pose:", tcp_pose)
# x = xAxis.get_position(Units.LENGTH_MILLIMETRES)
# y = yAxis.get_position(Units.LENGTH_MILLIMETRES)
# z = zAxis.get_position(Units.LENGTH_MILLIMETRES)

print("1) Start pupil tracking in a Terminal with -a localhost")
print("2) Manually move the zaber stages such that the eye is within 2 mm of the center")
input("Press Enter to continue when complete...")

# Set parameters
max_iter = 20
tol = 0.01 # mm


def correct_eye_position():
    # Moves eye to the center of the pupil tracking system with zaber stages
    dtol = 0.02;
    pos = get_pos(mode)
    d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
    while d > dtol:
        xAxis.move_relative(pos[0], Units.LENGTH_MILLIMETRES)
        yAxis.move_relative(pos[1], Units.LENGTH_MILLIMETRES)
        zAxis.move_relative(pos[2], Units.LENGTH_MILLIMETRES)
        time.sleep(0.5)
        pos = get_pos(mode)
        d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)

correct_eye_position()

iteration_history = []

for i in range(max_iter):

    print(f"Iteration {i+1} of {max_iter}")

    ## X-AXIS ALIGNMENT
   
    
    ## RY rotation
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    ry = tcp_pose[4]
    # Move to start position, +5 mm
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position 
    time.sleep(1)
    pos = get_pos(mode)
    y5 = pos[1]
    print(f"Current y position: {y5:.2f}")  
    # y5 = float(input("Enter current y position: ")) 
    # Jump position to -5 mm
    xAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    y_5 = pos[1]
    print(f"Current y position: {y_5:.2f}")
    # y_5 = float(input("Enter current y position: "))
    # Move back to center
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dy = y_5 - y5
    if abs(dy) > tol:
        d_theta = dy/10 # radians, small angle approx.
        # d_theta = d_theta *0.125
        ry = ry - d_theta
        
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], ry, tcp_pose[5]], 0.125, 0.125, True)
    correct_eye_position()



    ## RZ rotation
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    rz = tcp_pose[5]
    # Move to start position, +5 mm
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    z5 = pos[2]
    print(f"Current z position: {z5:.2f}")
    # Jump position to -5 mm
    xAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    z_5 = pos[2]
    print(f"Current z position: {z_5:.2f}")
    # Move back to center
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dz = z_5 - z5
    if abs(dz) > tol:
        d_theta = dz/10 # radians, small angle approx.
        # d_theta = d_theta  *0.125
        rz = rz - d_theta

    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], rz], 0.125, 0.125, True)
    correct_eye_position()

    ## Y-AXIS ALIGNMENT

    ## RY rotation
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    ry = tcp_pose[4]
    # Move to start position, +5 mm
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    x5 = pos[0]
    print(f"Current x position: {x5:.2f}")
    # Jump position to -5 mm
    yAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    x_5 = pos[0]
    print(f"Current x position: {x_5:.2f}")
    # Move back to center
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dx = x_5 - x5
    if abs(dx) > tol:
        d_theta = dx/10 # radians, small angle approx.
        ry = ry + d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], ry, tcp_pose[5]], 0.125, 0.125, True)
    correct_eye_position()

    ## RX rotation
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    rx = tcp_pose[3]
    # Move to start position, +5 mm
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    z5 = pos[2]
    print(f"Current z position: {z5:.2f}")
    # Jump position to -5 mm
    yAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    z_5 = pos[2]
    print(f"Current z position: {z_5:.2f}")
    # Move back to center
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dz = z_5 - z5
    if abs(dz) > tol:
        d_theta = dz/10 # radians, small angle approx.
        rx = rx + d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], rx, tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
    correct_eye_position()
    
    ## Z-AXIS ALIGNMENT

    ## RZ rotation
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    rz = tcp_pose[5]
    # Move to start position, +5 mm
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    x5 = pos[0]
    print(f"Current x position: {x5:.2f}")
    # Jump position to -5 mm
    zAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    x_5 = pos[0]
    print(f"Current x position: {x_5:.2f}")
    # Move back to center
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dx = x_5 - x5
    if abs(dx) > tol:
        d_theta = dx/10 # radians, small angle approx.
        rz = rz + d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], rz], 0.125, 0.125, True)
    correct_eye_position()

    ## RX rotation
    # Get current relative rotations
    tcp_pose = rtde_r.getActualTCPPose()
    rx = tcp_pose[3]
    # Move to start position, +5 mm
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    y5 = pos[1]
    print(f"Current y position: {y5:.2f}")
    # Jump position to -5 mm
    zAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    # Record pupil position
    time.sleep(1)
    pos = get_pos(mode)
    y_5 = pos[1]
    print(f"Current y position: {y_5:.2f}")
    # Move back to center
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    # Calculate differences in pupil position and the corresponding rotation adjustments
    dy = y_5 - y5
    if abs(dy) > tol:
        d_theta = dy/10 # radians, small angle approx.
        rx = rx - d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], rx, tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
    correct_eye_position()

    # Check Iteration Error
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    y5 = pos[1]
    xAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    y_5 = pos[1]
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dy1 = y_5 - y5
    correct_eye_position()

    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    z5 = pos[2]
    xAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    z_5 = pos[2]
    xAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dz1 = z_5 - z5
    correct_eye_position()

    # Y-axis: response in x for RY and z for RX
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    x5 = pos[0]
    yAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    x_5 = pos[0]
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dx1 = x_5 - x5
    correct_eye_position()

    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    z5 = pos[2]
    yAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    z_5 = pos[2]
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dz2 = z_5 - z5
    correct_eye_position()

    # Z-axis: response in x for RZ and y for RX
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    x5 = pos[0]
    zAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    x_5 = pos[0]
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dx2 = x_5 - x5
    correct_eye_position()

    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    y5 = pos[1]
    zAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    y_5 = pos[1]
    zAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dy2 = y_5 - y5
    correct_eye_position()

    d_values = {
        "dx": dx,
        "dy": dy,
        "dy1": dy1,
        "dz1": dz1,
        "dx1": dx1,
        "dz2": dz2,
        "dx2": dx2,
        "dy2": dy2,
    }
    for label, value in d_values.items():
        print(f"{label}: {value:.2f}")
    d_avg = sum(abs(value) for value in d_values.values()) / len(d_values)
    print(f"Average absolute d: {d_avg:.3f}")

    tcp_pose = rtde_r.getActualTCPPose()
    current_angles = [tcp_pose[3], tcp_pose[4], tcp_pose[5]]
    iteration_history.append({
        "iteration": i + 1,
        "d_avg": d_avg,
        "angles": current_angles.copy(),
        "ds": d_values.copy()
    })

best_iteration = min(iteration_history, key=lambda x: x["d_avg"])
print("Best iteration: {}".format(best_iteration))

# Save best iteration d-values to CSV with labels
filename_timestamp = time.strftime("%Y%m%d_%H%M%S")
best_csv_file = f"best_iteration_ds_{filename_timestamp}.csv"
with open(best_csv_file, "w", newline="") as csv_file:
    csv_file.write("label,value\n")
    for label, value in best_iteration["ds"].items():
        csv_file.write(f"{label},{value}\n")
print(f"Saved best iteration ds values to {best_csv_file}")

if best_iteration["iteration"] != len(iteration_history):
    best_angles = best_iteration["angles"]
    current_tcp_pose = rtde_r.getActualTCPPose()
    target_tcp_pose = [
        current_tcp_pose[0],
        current_tcp_pose[1],
        current_tcp_pose[2],
        best_angles[0],
        best_angles[1],
        best_angles[2],
    ]
    print(f"Moving robot to best iteration {best_iteration['iteration']} angles: {best_angles}")
    rtde_c.moveL(target_tcp_pose, 0.125, 0.125, True)

# Plot d_avg over all iterations
iterations = [entry["iteration"] for entry in iteration_history]
d_history = [entry["d_avg"] for entry in iteration_history]
plt.figure()
plt.plot(iterations, d_history, marker='o', linestyle='-')
plt.title("Average displacement per iteration")
plt.xlabel("Iteration")
plt.ylabel("d_avg (mm)")
plt.grid(True)
plt.tight_layout()
output_file = f"d_avg_history_{filename_timestamp}.png"
plt.savefig(output_file)
print(f"Saved d_avg plot to {output_file}")
plt.close()



correct_eye_position()

rtde_c.disconnect()
rtde_r.disconnect()





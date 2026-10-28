## Auto-alignment of the UR5e robot with the Zaber stage y-axis using pupil tracking
# One axis at a time offers better alignment, as zaber stages are not perfectly orthogonal
#
# Aislinn Hurley 10/28/2026

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
max_iter = 30
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
        time.sleep(1)
        pos = get_pos(mode)
        d = np.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)

correct_eye_position()

iteration_history = []

for i in range(max_iter):

    print(f"Iteration {i+1} of {max_iter}")

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
        # d_theta = d_theta*0.75
        ry = ry + d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], ry, tcp_pose[5]], 0.125, 0.125, True)
    time.sleep(1)
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
        d_theta = dz/10 # radians, small angle approx.\
        # d_theta = d_theta*0.75
        rx = rx + d_theta
    # Adjust robot rotation
    rtde_c.moveL([tcp_pose[0], tcp_pose[1], tcp_pose[2], rx, tcp_pose[4], tcp_pose[5]], 0.125, 0.125, True)
    time.sleep(1)
    correct_eye_position()
    
    # Check Iteration Error
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    x5 = pos[0]
    yAxis.move_relative(-10.0, Units.LENGTH_MILLIMETRES)
    time.sleep(1)
    pos = get_pos(mode)
    x_5 = pos[0]
    yAxis.move_relative(5.0, Units.LENGTH_MILLIMETRES)
    dx = x_5 - x5
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
    dz = z_5 - z5
    correct_eye_position()

    d_values = {
        "dx": dx,
        "dz": dz,
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
best_csv_file = f"best_iteration_ds_y_{filename_timestamp}.csv"
with open(best_csv_file, "w", newline="") as csv_file:
    csv_file.write("label,value\n")
    for label, value in best_iteration["ds"].items():
        csv_file.write(f"{label},{value}\n")
print(f"Saved best iteration ds values to {best_csv_file}")

best_angles = best_iteration["angles"]
if best_iteration["iteration"] != len(iteration_history):
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
output_file = f"d_avg_history_y_{filename_timestamp}.png"
plt.savefig(output_file)
print(f"Saved d_avg plot to {output_file}")
plt.close()

correct_eye_position()

# Ask whether final AngularScan3DRAOCT setup is needed. If so, retrieve input values
convert_for_angular_scan = ""
while convert_for_angular_scan not in ("y", "n"):
    convert_for_angular_scan = input("Are you going to use AngularScan3DRAOCT? (y/n): ").strip().lower()

if convert_for_angular_scan == "y":
    print("Switch robot to local mode and run rotvec2rpy to get roll, pitch, yaw of aligned position")
    roll = float(input("Enter roll (radians): "))
    pitch = float(input("Enter pitch (radians): "))
    yaw = float(input("Enter yaw (radians): "))

    axes_pitch = roll - 1.568
    axes_roll = pitch - 0.00639
    axes_yaw = yaw - 0.00125

    print(f"Calculated axes pitch: {axes_pitch:.6f}, roll: {axes_roll:.6f}, yaw: {axes_yaw:.6f}")

    axes_csv_file = f"angular_scan_axes_y_{filename_timestamp}.csv"
    with open(axes_csv_file, "w", newline="") as csv_file:
        csv_file.write("axis,value\n")
        csv_file.write(f"pitch,{axes_pitch}\n")
        csv_file.write(f"roll,{axes_roll}\n")
        csv_file.write(f"yaw,{axes_yaw}\n")
    print(f"Saved AngularScan3DRAOCT axes values to {axes_csv_file}")
else:
    print("Axes are aligned")

rtde_c.disconnect()
rtde_r.disconnect()

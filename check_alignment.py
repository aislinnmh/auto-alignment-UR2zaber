## Check current alignment, moving only the zaber stage

import time
import matplotlib

from align_axes import correct_eye_position
matplotlib.use("Agg")
import matplotlib.pyplot as plt

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

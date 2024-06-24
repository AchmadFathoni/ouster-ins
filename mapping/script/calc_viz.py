"""
Visualize ros2 bag odometry in 3D
usage: python3 calc_viz.py {bag_file}
"""

import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d
from tf_transformations import euler_from_quaternion
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

#Linearize warped angle to eliminate discontinuity
def linearize(angles : list[float]):
    last_discontinuity = None
    for i, angle in enumerate(angles):
        if i == 0: continue
        angle_prev = angles[i-1]
        if last_discontinuity:
            if abs(angle - angles[last_discontinuity]) < 180: 
                last_discontinuity = None
            else:
                angle_prev = angles[last_discontinuity]
        if angle - angle_prev < -180.0:
            angles[i] += 360.0
            last_discontinuity = i
        elif angle - angle_prev > 180.0:
            angles[i] -= 360.0
            last_discontinuity = i

# Read the rosbag file
bag_file = sys.argv[1]
typestore = get_typestore(Stores.ROS2_HUMBLE)
x ,y, z =  [], [], []
r ,p, yw =  [], [], []
with Reader(bag_file) as reader:
    connections = [x for x in reader.connections if x.topic == '/odom_ins_enu']
    for connection, timestamp, rawData in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawData, connection.msgtype).pose.pose # type: ignore
        pos = msg.position
        orientation = msg.orientation
        x.append(pos.x)
        y.append(pos.y)
        z.append(pos.z)

        orientation_list= [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        r.append(np.rad2deg(roll))
        p.append(np.rad2deg(pitch))
        yw.append(np.rad2deg(yaw))

# Plot
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.set_box_aspect([np.ptp(x),np.ptp(y), np.ptp(z)])
ax.plot(x, y, z, label='Odometry')

sigma = 30
x = gaussian_filter1d(np.array(x), sigma=sigma)
y = gaussian_filter1d(np.array(y), sigma=sigma)
z = gaussian_filter1d(np.array(z), sigma=5)
ax.plot(x, y, z ,label='Filtered Odometry')
ax.legend()

ax2 = fig.add_subplot(322)
ax2.plot(r,label='Roll')
# r = gaussian_filter1d(np.array(r), sigma=0.5)
# ax2.plot(r,label='Filtered roll')
ax2.legend()

ax3 = fig.add_subplot(324)
ax3.plot(p,label='Pitch')
# p = gaussian_filter1d(np.array(p), sigma=0.5)
# ax3.plot(p,label='Filtered Pitch')
ax3.legend()

ax4 = fig.add_subplot(326)
ax4.plot(yw,label='Yaw')

linearize(yw)
yw = np.array(yw)
# yw = gaussian_filter1d(yw, sigma=10)
ax4.plot(yw,label='Filtered Yaw')
ax4.legend()


plt.show()

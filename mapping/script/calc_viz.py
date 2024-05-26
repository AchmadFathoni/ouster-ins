"""
Visualize ros2 bag odometry in 3D
usage: python3 calc_viz.py {bag_file}
"""

import sys
import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# Read the rosbag file
bag_file = sys.argv[1]
typestore = get_typestore(Stores.ROS2_HUMBLE)
x ,y, z =  [], [], []
with Reader(bag_file) as reader:
    connections = [x for x in reader.connections if x.topic == '/odom_ins_enu']
    for connection, timestamp, rawData in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawData, connection.msgtype).pose.pose.position
        x.append(msg.x)
        y.append(msg.y)
        z.append(msg.z)

# Plot
ax = plt.figure().add_subplot(projection='3d')
ax.set_box_aspect([np.ptp(x),np.ptp(y), np.ptp(z)])
ax.plot(x, y, z, label='Odometry')
ax.legend()

plt.show()

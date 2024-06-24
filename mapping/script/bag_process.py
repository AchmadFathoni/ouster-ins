"""
usage: python3 reindex my_bag
Reindex ROS message by message's timestamp and offset the GPS using first GPS position message.
Convert and filter odom message to tf. Filter is Gaussian non causal filter.
"""
import sys
import numpy as np
from scipy.ndimage import gaussian_filter1d
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

lidar_topic = '/filtered'
ins_topic = '/odom_ins_enu'
typestore = get_typestore(Stores.ROS2_HUMBLE)
Pointcloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
Tf2 = typestore.types['tf2_msgs/msg/TFMessage']
Transform= typestore.types['geometry_msgs/msg/Transform']
TransformStamped = typestore.types['geometry_msgs/msg/TransformStamped']
Vector = typestore.types['geometry_msgs/msg/Vector3']
Quat = typestore.types['geometry_msgs/msg/Quaternion']
first_point = {"x": 0, "y": 0, "z": 0, "initiated": False}

bag_file = sys.argv[1]

def header_time(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec

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

with Reader(bag_file) as reader, Writer(bag_file+'_reindex') as writer:
    tf_conn = writer.add_connection("/tf", Tf2.__msgtype__, typestore=typestore)
    lidar_conn = writer.add_connection(lidar_topic, Pointcloud2.__msgtype__, typestore=typestore)
    x, y, z, r, p, ya = [[], [], [], [], [], []]
    msg_headers = []
    for conn, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        if conn.topic == ins_topic:
            position = msg.pose.pose.position # type: ignore
            if not first_point["initiated"]:
                first_point["x"] = position.x
                first_point["y"] = position.y
                first_point["z"] = position.z
                first_point["initiated"] = True
            x.append(position.x - first_point["x"])
            y.append(position.y - first_point["y"])
            z.append(position.z - first_point["z"])

            q = msg.pose.pose.orientation # type: ignore
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            r.append(roll)
            p.append(pitch)
            ya.append(yaw)

            msg_headers.append(msg.header) # type: ignore
        elif conn.topic == lidar_topic:
            writer.write(lidar_conn, header_time(msg.header), rawdata) # type: ignore

    #Gaussian filter
    x = gaussian_filter1d(np.array(x), sigma=30)
    y = gaussian_filter1d(np.array(y), sigma=30)
    z = gaussian_filter1d(np.array(z), sigma=5)
    # Roll and pitch of INS is pretty good, often doesn't need gaussian filter.
    # r = gaussian_filter1d(np.array(r), sigma=0.5)
    # p = gaussian_filter1d(np.array(p), sigma=0.5)
    linearize(ya)
    ya = gaussian_filter1d(np.array(ya), sigma=20)

    #Konversi ke dari odom ke tf2
    for x_, y_, z_, r_, p_, ya_, msg_header in zip(x, y, z, r, p, ya, msg_headers):
        q_x, q_y, q_z, q_w = quaternion_from_euler(r_, p_, ya_)
        q_msg = Quat(x=q_x, y=q_y, z=q_z, w=q_w)
        v_msg = Vector(x_, y_, z_)
        t_msg = Transform(translation=v_msg, rotation=q_msg)
        ts_msg = TransformStamped(header=msg_header, child_frame_id="base_link", transform=t_msg) # type: ignore
        tf2_msg = Tf2(transforms=[ts_msg])
        writer.write(tf_conn, header_time(msg_header), typestore.serialize_cdr(tf2_msg, Tf2.__msgtype__))

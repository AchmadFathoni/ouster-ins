"""
usage: python3 reindex my_bag
This will reindex ROS message by message's timestamp and offset the GPS using first GPS position message.
depend on https://pypi.org/project/rosbags 0.9.22
"""
import sys

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
tf2_static_msg = []
first_point = {"x": 0, "y": 0, "z": 0, "initiated": False}
first_time = 0

bag_file = sys.argv[1]
with Reader(bag_file) as reader, Writer(bag_file+'_reindex') as writer:
    tf_conn = writer.add_connection("/tf", Tf2.__msgtype__, typestore=typestore)
    lidar_conn = writer.add_connection(lidar_topic, Pointcloud2.__msgtype__, typestore=typestore)
    for conn, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        time = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if conn.topic == ins_topic:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            if not first_point["initiated"]:
                first_point["x"] = position.x
                first_point["y"] = position.y
                first_point["z"] = position.z
                first_point["initiated"] = True
                first_time = time
            position.x -= first_point["x"]
            position.y -= first_point["y"]
            position.z -= first_point["z"]

            #Konversi ke dari odom ke tf2 
            q_msg = Quat(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
            v_msg = Vector(x=position.x, y=position.y, z=position.z)
            t_msg = Transform(translation=v_msg, rotation=q_msg)
            ts_msg = TransformStamped(header=msg.header, child_frame_id="base_link", transform=t_msg)
            tf2_msg = Tf2(transforms=[ts_msg])
            writer.write(tf_conn, time, typestore.serialize_cdr(tf2_msg, Tf2.__msgtype__))
        elif conn.topic == lidar_topic:
            writer.write(lidar_conn, time, rawdata)

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
Odometry = typestore.types['nav_msgs/msg/Odometry']
PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']

class BagWriter:
    def __init__(self, writer, topic, type):
        self.connection = writer.add_connection(topic, type.__msgtype__, typestore = typestore)
        self.writer = writer
        self.topic = topic
        self.type = type
        self.first_point = {"x": 0, "y": 0, "z": 0, "initiated": False}

    def write(self, connection, rawdata):
        if connection.topic == self.topic:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            if self.topic == ins_topic:
                position = msg.pose.pose.position
                if not self.first_point["initiated"]:
                    self.first_point["x"] = position.x
                    self.first_point["y"] = position.y
                    self.first_point["z"] = position.z
                    self.first_point["initiated"] = True
                position.x -= self.first_point["x"]
                position.y -= self.first_point["y"]
                position.z -= self.first_point["z"]
                msg.pose.pose.position = position
                msg.child_frame_id = "base_link"
            self.writer.write(self.connection, time, typestore.serialize_cdr(msg, connection.msgtype))

bag_file = sys.argv[1]
with Reader(bag_file) as reader, Writer(bag_file+'_reindex') as writer:
    lidar_writer = BagWriter(writer, lidar_topic, PointCloud2)
    ins_writer = BagWriter(writer, ins_topic, Odometry)
    for conn, timestamp, rawdata in reader.messages():
        lidar_writer.write(conn, rawdata)
        ins_writer.write(conn, rawdata)

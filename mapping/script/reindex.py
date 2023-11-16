#usage: python3 reindex my_bag
#this will reindex ROS message by message's timestamp and offset the GPS using first GPS position message
#depend on https://pypi.org/project/rosbags
import sys

from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as PointCloud2
from rosbags.typesys.types import nav_msgs__msg__Odometry as Odometry

lidar_topic = '/ouster/points'
ins_topic = '/odom_ins_enu'

class BagWriter:
    def __init__(self, writer, topic, type):
        self.connection = writer.add_connection(topic, type)
        self.writer = writer
        self.topic = topic
        self.type = type
        self.first_point = {"x": 0, "y": 0, "z": 0, "initiated": False}

    def write(self, connection, rawdata):
        if connection.topic == self.topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
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
            self.writer.write(self.connection, time, serialize_cdr(msg, self.type))

bag_file = sys.argv[1]
with Reader(bag_file) as reader, Writer(bag_file+'_reindex') as writer:
    lidar_writer = BagWriter(writer, lidar_topic, PointCloud2.__msgtype__)
    ins_writer = BagWriter(writer, ins_topic, Odometry.__msgtype__)
    for conn, timestamp, rawdata in reader.messages():
        lidar_writer.write(conn, rawdata)
        ins_writer.write(conn, rawdata)

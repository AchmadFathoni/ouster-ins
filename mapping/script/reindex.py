import sys

from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as PointCloud2
from rosbags.typesys.types import nav_msgs__msg__Odometry as Odometry

class BagWriter:
    def __init__(self, writer, topic, type):
        self.connection = writer.add_connection(topic, type, 'cdr', '')
        self.writer = writer
        self.topic = topic
        self.type = type

    def write(self, connection, rawdata):
        if connection.topic == self.topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            self.writer.write(self.connection, time, serialize_cdr(msg, self.type))

bag_file = sys.argv[1]
with Reader(bag_file) as reader, Writer(bag_file+'_reindex') as writer:
    lidar_writer = BagWriter(writer, '/ouster/points', PointCloud2.__msgtype__)
    ins_writer = BagWriter(writer, '/odom_ins_enu', Odometry.__msgtype__)
    for conn, timestamp, rawdata in reader.messages():
        lidar_writer.write(conn, rawdata)
        ins_writer.write(conn, rawdata)

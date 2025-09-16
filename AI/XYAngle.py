import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import utilities

import sys

delta_dist = 0.5
delta_angle = 180.0

if len(sys.argv) < 2:
    rosbag_path = "../rosbags/map1_run2/map1_run2_0.db3"
else:
    rosbag_path = "../rosbags/" + sys.argv[1] + "/" + sys.argv[1] + "_0.db3"


storage_options = rosbag2_py.StorageOptions(uri=rosbag_path, storage_id="sqlite3")
conventer_options = rosbag2_py.ConverterOptions("","")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options,conventer_options)

type_map = {
    "/scan": LaserScan,
    "/robot_pose": PoseStamped
}

scans = []
poses = []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic in type_map:
        msg_type = type_map[topic]
        msg = deserialize_message(data, msg_type)

        if topic == "/scan":
            scans.append((t, np.array(msg.ranges, dtype = np.float32)))
        elif topic == "/robot_pose":
            yaw = utilities.quaternion_to_yaw(msg.pose.orientation)
            poses.append((t, np.array([msg.pose.position.x,
                                      msg.pose.position.y,
                                      yaw], dtype = np.float32)))
            
print(f"Collected {len(scans)} scans and {len(poses)} positions.")


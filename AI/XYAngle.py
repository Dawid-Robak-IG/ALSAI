import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import utilities

import sys
from colorama import Fore, Style, init

init(autoreset=True)
GREEN = "\033[32m"

DELTA_DIST = 0.5
DELTA_ANGLE = 180.0

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
            
print(Fore.GREEN + f"Collected {len(scans)} scans and {len(poses)} positions.")

xya_scan_data = []
scans_pars_data = []
reallocates = []


for t, scan in scans:
    pose = utilities.get_pose_for_scan(t, poses)
    xya_scan_data.append((scan,pose))

for i in range(len(xya_scan_data)):
    scan1,pose1 = xya_scan_data[i]
    x1,y1,yaw1 = pose1

    for j in range(i+1, len(xya_scan_data)):
       scan2,pose2 = xya_scan_data[j]
       x2,y2,yaw2 = pose2 

       dx = x2 - x1
       dy = y2 - y1
       dist = np.sqrt(dx**2 + dy**2)

       if dist <= DELTA_DIST:
           delta_yaw = np.arctan2(np.sin(yaw2-yaw1),np.cos(yaw2-yaw1))
           if abs(delta_yaw) <= DELTA_ANGLE:
               scans_pars_data.append((scan1,scan2))
               reallocates.append([dx,dy,delta_yaw])


X = np.stack([np.stack([s1,s2], axis=-1) for s1,s2 in scans_pars_data])
y = np.array(reallocates, dtype=np.float32)

np.savez("dataset.npz", X=X,y=y)
print(Fore.GREEN + "Dataset ready: ",X.shape, y.shape)
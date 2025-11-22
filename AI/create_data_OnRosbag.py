import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosbags.rosbag1 import Reader
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import utilities

from colorama import Fore, Style, init
import sys, os
import traceback
from pathlib import Path


def create_data_file(rosbag):
    rosbag_path = os.path.expanduser(f"~/ALSAI/rosbags/{rosbag}")
    print(Fore.GREEN + "Got rosbag: ", rosbag_path)
    init(autoreset=True)

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
                scan = np.array(msg.ranges, dtype=np.float32)
                scan = np.nan_to_num(scan, nan=0.0, posinf=0.0, neginf=0.0)
                scans.append((t, scan))

            elif topic == "/robot_pose":
                yaw = utilities.quaternion_to_yaw(msg.pose.orientation)
                poses.append((t, np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        yaw], dtype = np.float32)))
                
    print(Fore.GREEN + f"Collected {len(scans)} scans and {len(poses)} positions.")

    scan_transformation_data = []
    scan_transformation_pairs = []


    for t, scan in scans:
        pose = utilities.get_pose_for_scan(t, poses)
        scan_transformation_data.append((scan,pose))

    
    for trans_idx in range( len(scan_transformation_data) - max(utilities.OFFSETs_IDX)):
        for i in utilities.OFFSETs_IDX:
            data = utilities.is_data_near(scan_transformation_data[trans_idx][1], scan_transformation_data[trans_idx+i][1])
            d_trans = [data["dx"],data["dy"],data["dyaw"]]

            if data["is_near"]:
                scan_transformation_pairs.append(
                    ( (scan_transformation_data[trans_idx][0],
                      scan_transformation_data[trans_idx+i][0]), d_trans )
                )

    print(Fore.GREEN + f"Got pairs: {len(scan_transformation_pairs)}")

    #noise
    if rosbag not in ["map5_run1.npz", "map6_run1.npz"]:
        scan_transformation_pairs = utilities.make_gaussian_noise(scan_transformation_pairs, noise=0.1)
        print(Fore.GREEN + "Added gaussian noise")

        num_to_cut = int(len(scan_transformation_pairs) * utilities.CUT_FRACTION)
        indices_to_cut = np.random.choice(len(scan_transformation_pairs), num_to_cut, replace=False)

        with_cut = [scan_transformation_pairs[i] for i in indices_to_cut]
        without_cut = [scan_transformation_pairs[i] for i in range(len(scan_transformation_pairs)) if i not in indices_to_cut]

        cut_pairs = utilities.cut_data_from_scans(with_cut, max_points_to_cut=20)
        print(Fore.GREEN + f"Cut data from {utilities.CUT_FRACTION * 100}\% of scan pairs")
        scan_transformation_pairs = without_cut+cut_pairs

    output_folder = os.path.expanduser(f"~/ALSAI/data/single_map_data")
    os.makedirs(output_folder, exist_ok=True)

    output_file = os.path.expanduser(f"~/ALSAI/data/single_map_data/{rosbag}")

    np.savez_compressed(output_file, pairs=np.array(scan_transformation_pairs, dtype=object))
    print(Fore.GREEN + f"Saved {len(scan_transformation_pairs)} pairs to {rosbag_path}")

def create_data_file_Melodic(rosbag_filename):
    path_str = os.path.expanduser(f"~/ALSAI/jetbot/{rosbag_filename}")
    rosbag_path = Path(path_str)
    
    print("Got rosbag: ", rosbag_path)

    typestore = get_typestore(Stores.ROS1_NOETIC)
    scans = []
    poses = []

    try:
        with AnyReader([rosbag_path], default_typestore=typestore) as reader:
            connections = [x for x in reader.connections if x.topic in ['/scan', '/r1/pose']]

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)

                if connection.topic == "/scan":
                    scan = np.array(msg.ranges, dtype=np.float32)
                    scan = np.nan_to_num(scan, nan=0.0, posinf=0.0, neginf=0.0)
                    scans.append((timestamp, scan))

                elif connection.topic == "/r1/pose":
                    yaw = utilities.quaternion_to_yaw(msg.pose.orientation)
                    poses.append((timestamp, np.array([
                                            msg.pose.position.x,
                                            msg.pose.position.y,
                                            yaw], dtype=np.float32)))

    except Exception as e:
        print(Fore.RED + f"Error processing bag: {e}")
        traceback.print_exc()
        sys.exit(1)
                
    print(Fore.GREEN + f"Collected {len(scans)} scans and {len(poses)} positions.")
    
    scan_transformation_data = []
    scan_transformation_pairs = []

    for t, scan in scans:
        pose = utilities.get_pose_for_scan(t, poses)
        scan_transformation_data.append((scan, pose))

    for trans_idx in range(len(scan_transformation_data) - max(utilities.OFFSETs_IDX)):
        for i in utilities.OFFSETs_IDX:
            idx_curr = trans_idx
            idx_next = trans_idx + i
            
            if idx_next >= len(scan_transformation_data):
                continue

            pose_curr = scan_transformation_data[idx_curr][1]
            pose_next = scan_transformation_data[idx_next][1]

            if pose_curr is None or pose_next is None:
                continue

            data = utilities.is_data_near(pose_curr, pose_next)
            d_trans = [data["dx"], data["dy"], data["dyaw"]]

            if data["is_near"]:
                scan_transformation_pairs.append(
                    ((scan_transformation_data[idx_curr][0],
                      scan_transformation_data[idx_next][0]), d_trans)
                )

    print(Fore.GREEN + f"Got pairs: {len(scan_transformation_pairs)}")

    output_folder = os.path.expanduser(f"~/ALSAI/data/single_map_data")
    os.makedirs(output_folder, exist_ok=True)

    output_filename = rosbag_filename.replace('.bag', '').replace('.npz', '')
    output_file = os.path.join(output_folder, output_filename)

    np.savez_compressed(output_file, pairs=np.array(scan_transformation_pairs, dtype=object))
    print(Fore.GREEN + f"Saved {len(scan_transformation_pairs)} pairs to {output_file}.npz")
    init(autoreset=True)


def main():
    init(autoreset=True)
    if len(sys.argv) == 2:
        create_data_file(sys.argv[1])
    if len(sys.argv) > 2 and sys.argv[2] == "melodic":
        print(Fore.LIGHTCYAN_EX + "Reading melodic bag")
        create_data_file_Melodic(sys.argv[1])

if __name__ == "__main__":
    main()
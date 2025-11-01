import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import utilities

from colorama import Fore, Style, init
import sys, os


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
    init(autoreset=True)


    scan_transformation_data = []
    scan_transformation_pairs = []


    for t, scan in scans:
        pose = utilities.get_pose_for_scan(t, poses)
        scan_transformation_data.append((scan,pose))

    
    for idx in utilities.OFFSETs_IDX:
        for i in range(utilities.OFFSET_DATA):
            data = utilities.is_data_near(scan_transformation_data[idx][1], scan_transformation_data[idx+i+1][1])
            d_trans = [data["dx"],data["dy"],data["dyaw"]]

            if data["is_near"]:
                scan_transformation_pairs.append(
                    ( (scan_transformation_data[idx][0],
                      scan_transformation_data[idx+i+1][0]), d_trans )
                )

    print(Fore.GREEN + f"Got pairs: {len(scan_transformation_pairs)}")
    init(autoreset=True)

    if rosbag not in ["map5_run1.npz", "map6_run1.npz"]:
        scan_transformation_pairs = utilities.make_gaussian_noise(scan_transformation_pairs, noise=0.1)

        num_to_cut = int(len(scan_transformation_pairs) * utilities.CUT_FRACTION)
        indices_to_cut = np.random.choice(len(scan_transformation_pairs), num_to_cut, replace=False)

        with_cut = [scan_transformation_pairs[i] for i in indices_to_cut]
        without_cut = [scan_transformation_pairs[i] for i in range(len(scan_transformation_pairs)) if i not in indices_to_cut]

        cut_pairs = utilities.cut_data_from_scans(with_cut, max_points_to_cut=20)
        scan_transformation_pairs = without_cut+cut_pairs



    output_folder = os.path.expanduser(f"~/ALSAI/data/single_map_data")
    os.makedirs(output_folder, exist_ok=True)

    output_file = os.path.expanduser(f"~/ALSAI/data/single_map_data/{rosbag}")

    np.savez_compressed(output_file, pairs=np.array(scan_transformation_pairs, dtype=object))
    print(Fore.GREEN + f"Saved {len(scan_transformation_pairs)} pairs to {rosbag_path}")
    init(autoreset=True)



def main():
    if len(sys.argv) > 1:
        create_data_file(sys.argv[1])

if __name__ == "__main__":
    main()
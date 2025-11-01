import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tensorflow.keras.models import Sequential # type: ignore
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input # type: ignore
from sklearn.model_selection import train_test_split
from tensorflow.keras.callbacks import EarlyStopping # type: ignore
from tensorflow.keras.models import load_model
import matplotlib.pyplot as plt

import utilities, os

from colorama import Fore, init

def test_on_npz(data, model_path, model_name):
    model = load_model(model_path)

    init(autoreset=True)

    all_scan_pairs = [pair[0] for pair in data]
    all_delta_transformation = [[pair[1] for pair in data]]

    X = np.stack([np.stack([s1,s2], axis=-1) for s1,s2 in all_scan_pairs])
    X = np.nan_to_num(X)
    y = np.array(all_delta_transformation, dtype=np.float32)
    if y.shape[0] == 1 and len(y.shape) == 3:
        y = y[0]

    print(Fore.GREEN + "Dataset ready: ",X.shape, y.shape)

    model.compile(optimizer='adam', loss='mse')
    model.summary()
    y_pred = model.predict(X)

    errors = y - y_pred
    err_x = errors[:,0]
    err_y = errors[:,1]
    err_theta = errors[:,2]

    plt.figure(figsize=(16,4))
    plt.subplot(1,3,1)
    utilities.plot_error_with_gaussian(plt, err_x, "Błąd Δx", "Wartość błędu [m]")
    plt.xlim(-0.3,0.3)

    plt.subplot(1,3,2)
    utilities.plot_error_with_gaussian(plt, err_y, "Błąd Δy", "Wartość błędu [m]")
    plt.xlim(-0.3,0.3)

    plt.subplot(1,3,3)
    utilities.plot_error_with_gaussian(plt, err_theta, "Błąd Δθ", "Wartość błędu [rad]")
    plt.xlim(-0.3,0.3)

    output_file = os.path.expanduser(f"~/ALSAI/tests_figs/{model_name}.png")
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(Fore.GREEN + f"Wykres zapisany jako: {output_file}")

    # plt.show()

def test_on_rosbag(rosbag_path, model_path):
    model = load_model(model_path)

    init(autoreset=True)

    DELTA_DIST = 0.5
    DELTA_ANGLE = np.deg2rad(180.0)

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
    X = np.nan_to_num(X)
    y = np.array(reallocates, dtype=np.float32)

    print(Fore.GREEN + "Dataset ready: ",X.shape, y.shape)

    model.compile(optimizer='adam', loss='mse')
    model.summary()
    y_pred = model.predict(X)

    errors = y - y_pred
    err_x = errors[:,0]
    err_y = errors[:,1]
    err_theta = errors[:,2]

    plt.figure(figsize=(16,4))
    plt.subplot(1,3,1)
    utilities.plot_error_with_gaussian(plt, err_x, "Błąd Δx", "Wartość błędu [m]")

    plt.subplot(1,3,2)
    utilities.plot_error_with_gaussian(plt, err_y, "Błąd Δy", "Wartość błędu [m]")

    plt.subplot(1,3,3)
    utilities.plot_error_with_gaussian(plt, err_theta, "Błąd Δθ", "Wartość błędu [rad]")

    plt.show()

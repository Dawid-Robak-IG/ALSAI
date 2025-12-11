import numpy as np
import matplotlib.pyplot as plt
import os
from colorama import init, Fore
import tensorflow as tf
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import utilities
import sys, os
import traceback

def plot_trajectories(traj_true, traj_pred, traj_odom):
    plt.figure(figsize=(10, 10))
    plt.plot(traj_true[:, 0], traj_true[:, 1], label='Rzeczywista pozycja', color='blue', linewidth=3, alpha=0.7)
    plt.plot(traj_pred[:, 0], traj_pred[:, 1], label=f'Pozycja estymowana z sieci neuronowej', color='red', linestyle='--', linewidth=2)
    plt.plot(traj_odom[:, 0], traj_odom[:, 1], label='Odometria', color='green', linestyle=':', linewidth=2)
    
    plt.plot(traj_true[0, 0], traj_true[0, 1], 'o', color='blue', label='Start')
    plt.plot(traj_true[-1, 0], traj_true[-1, 1], 'x', color='blue', markersize=10, label='Koniec (GT)')

    plt.xlabel('Pozycja X [m]')
    plt.ylabel('Pozycja Y [m]')
    plt.title(f'Porównanie Trajektorii')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    save_path = os.path.expanduser(f"~/ALSAI/jetbot_figs")
    os.makedirs(save_path, exist_ok=True)
    fig_path = os.path.join(save_path, f'trajectories.png')
    plt.savefig(fig_path)
    print(Fore.CYAN + f"Trajektorie zapisane w: {fig_path}")
    plt.show()

def get_XY_test_tflite():
    init(autoreset=True)
    model_path = os.path.expanduser(f"~/ALSAI/AI/models/model_conv3_dropout_leaky.keras")
    data_file = os.path.expanduser(f"~/ALSAI/data/DR_INZ_PRZEJAZD1.npz")
    data_npz = np.load(data_file, allow_pickle=True)
    data = data_npz['arr_0']

    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    all_scan_pairs = [pair[0] for pair in data]
    all_delta_transformation = [[pair[1] for pair in data]]
    X = np.stack([np.stack([s1,s2], axis=-1) for s1,s2 in all_scan_pairs])
    X = np.nan_to_num(X)
    y = np.array(all_delta_transformation, dtype=np.float32)
    if y.shape[0] == 1 and len(y.shape) == 3:
        y = y[0]
    print(Fore.GREEN + "Dataset ready: ",X.shape, y.shape)

    y_pred = []
    for sample in X:
        sample = np.expand_dims(sample, axis=0).astype(np.float32)
        interpreter.set_tensor(input_details[0]['index'], sample)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])
        y_pred.append(output[0])
    y_pred = np.array(y_pred)

    return [X, y_pred]

def get_XY_motion_capture():
    path_str = os.path.expanduser(f"~/ALSAI/jetbot/DR_INZ_PRZEJAZD1.bag")
    rosbag_path = Path(path_str)
    
    print("Got rosbag: ", rosbag_path)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    poses = []

    try:
        with AnyReader([rosbag_path], default_typestore=typestore) as reader:
            connections = [x for x in reader.connections if x.topic in ['/r1/pose']]

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)

                if connection.topic == "/r1/pose":
                    poses.append(np.array([
                                    msg.pose.position.x,
                                    msg.pose.position.y], dtype=np.float32))

    except Exception as e:
        print(Fore.RED + f"Error processing bag: {e}")
        traceback.print_exc()
        sys.exit(1)

    return poses

def get_XY_odometry():
    path_str = os.path.expanduser(f"~/ALSAI/jetbot/DR_INZ_PRZEJAZD1.bag")
    rosbag_path = Path(path_str)
    
    print("Got rosbag: ", rosbag_path)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    poses = []

    try:
        with AnyReader([rosbag_path], default_typestore=typestore) as reader:
            connections = [x for x in reader.connections if x.topic in ['/odom']]

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)

                if connection.topic == "/odom":
                    poses.append(np.array([
                                    msg.pose.pose.position.x,
                                    msg.pose.pose.position.y], dtype=np.float32))

    except Exception as e:
        print(Fore.RED + f"Error processing bag: {e}")
        traceback.print_exc()
        sys.exit(1)

    return poses

def gen_traj():
    traj_true = get_XY_motion_capture()
    traj_pred = get_XY_test_tflite()
    traj_odom = get_XY_odometry()
    plot_trajectories(traj_true, traj_pred, traj_odom)

def main():
    gen_traj()

if __name__ == "__main__":
    main()
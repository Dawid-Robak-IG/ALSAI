import numpy as np
import matplotlib.pyplot as plt
import os
from colorama import init, Fore
import tensorflow as tf
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import utilities
import sys
import traceback

def reconstruct_trajectory(deltas):
    x, y, theta = 0.0, 0.0, 0.0
    trajectory = [[0.0, 0.0]]

    for delta in deltas:
        dx, dy, dtheta = delta[0], delta[1], delta[2]

        global_dx = dx * np.cos(theta) - dy * np.sin(theta)
        global_dy = dx * np.sin(theta) + dy * np.cos(theta)
        
        x += global_dx
        y += global_dy
        theta += dtheta
        
        trajectory.append([x, y])
        
    return np.array(trajectory)

def normalize_start_point(traj):
    traj = np.array(traj)
    if len(traj) > 0:
        start_x = traj[0, 0]
        start_y = traj[0, 1]
        traj[:, 0] -= start_x
        traj[:, 1] -= start_y
    return traj

def plot_trajectories(traj_true, traj_pred, traj_odom):
    traj_true = normalize_start_point(traj_true)
    traj_odom = normalize_start_point(traj_odom)
    traj_pred = np.array(traj_pred) 

    plt.figure(figsize=(10, 10))
    
    if len(traj_true) > 0:
        plt.plot(traj_true[:, 0], traj_true[:, 1], label='Ground Truth (MoCap)', color='blue', linewidth=3, alpha=0.6)
        plt.plot(traj_true[-1, 0], traj_true[-1, 1], 'x', color='blue', markersize=10, label='Koniec GT')
        
    if len(traj_pred) > 0:
        plt.plot(traj_pred[:, 0], traj_pred[:, 1], label='Sieć Neuronowa (Estymacja)', color='red', linestyle='--', linewidth=2)
        
    if len(traj_odom) > 0:
        plt.plot(traj_odom[:, 0], traj_odom[:, 1], label='Odometria Kołowa', color='green', linestyle=':', linewidth=2)
    
    plt.plot(0, 0, 'o', color='black', label='Start (0,0)')

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

def get_predictions_tflite():
    model_path = os.path.expanduser(f"~/ALSAI/AI/models/model_conv3_dropout_leaky.tflite")
    data_file = os.path.expanduser(f"~/ALSAI/data/DR_INZ_PRZEJAZD1.npz")
    print(Fore.GREEN + f"Got data file: {data_file}")
    
    data_npz = np.load(data_file, allow_pickle=True)
    data = data_npz['pairs']

    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    all_scan_pairs = [pair[0] for pair in data]
    
    X = np.stack([np.stack([s1,s2], axis=-1) for s1,s2 in all_scan_pairs])
    X = np.nan_to_num(X)
    
    print(Fore.GREEN + f"Dataset ready for inference: {X.shape}")

    y_pred_deltas = []
    print("Running inference...")
    for sample in X:
        sample = np.expand_dims(sample, axis=0).astype(np.float32)
        interpreter.set_tensor(input_details[0]['index'], sample)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])
        y_pred_deltas.append(output[0])
    
    return np.array(y_pred_deltas)

def get_XY_motion_capture():
    path_str = os.path.expanduser(f"~/ALSAI/jetbot/DR_INZ_PRZEJAZD1.bag")
    rosbag_path = Path(path_str)
    
    print("Got rosbag (MoCap): ", rosbag_path)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    poses = []

    try:
        with AnyReader([rosbag_path], default_typestore=typestore) as reader:
            topics = [x.topic for x in reader.connections if 'pose' in x.topic or 'odom' in x.topic]
            target_topic = "/r1/pose" 
            
            connections = [x for x in reader.connections if x.topic == target_topic]

            if not connections:
                print(Fore.YELLOW + f"Warning: Topic {target_topic} not found in bag. Available: {topics}")
                return []

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                poses.append([msg.pose.position.x, msg.pose.position.y])

    except Exception as e:
        print(Fore.RED + f"Error processing bag: {e}")
        return []

    return np.array(poses)

def get_XY_odometry():
    path_str = os.path.expanduser(f"~/ALSAI/jetbot/DR_INZ_PRZEJAZD1.bag")
    rosbag_path = Path(path_str)
    
    print("Got rosbag (Odom): ", rosbag_path)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    poses = []

    try:
        with AnyReader([rosbag_path], default_typestore=typestore) as reader:
            connections = [x for x in reader.connections if x.topic == '/odom']
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                poses.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    except Exception as e:
        print(Fore.RED + f"Error processing bag: {e}")
        return []

    return np.array(poses)

def gen_traj():
    init(autoreset=True)
    
    traj_true_raw = get_XY_motion_capture()
    traj_odom_raw = get_XY_odometry()
    deltas_pred = get_predictions_tflite()
    
    print("Reconstructing trajectory from neural network deltas...")
    traj_pred_path = reconstruct_trajectory(deltas_pred)

    plot_trajectories(traj_true_raw, traj_pred_path, traj_odom_raw)

def main():
    gen_traj()

if __name__ == "__main__":
    main()
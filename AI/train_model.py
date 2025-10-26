import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tensorflow.keras.models import Sequential # type: ignore
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input # type: ignore
from sklearn.model_selection import train_test_split
from tensorflow.keras.callbacks import EarlyStopping # type: ignore
import matplotlib.pyplot as plt

import train_network
import build_model

import sys
from colorama import Fore, Style, init
import os


def train_model(model_name):
    init(autoreset=True)

    model_path = os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.keras")

    if not os.path.exists(model_path):
        print(Fore.GREEN + f"Building model for name: {model_name}")
        model = build_model.build_model(model_name)
        model.save(os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.keras"))
    else:
        print(Fore.YELLOW + "Trained model for this name already exist, exiting...")
        sys.exit(2)

    data_file = os.path.expanduser(f"~/ALSAI/data/train_data.npz")
    data_npz = np.load(data_file, allow_pickle=True)
    data = data_npz['arr_0']

    train_network.train_on_npz(data, model_path)

def main():
    if len(sys.argv) > 1:
        model_name = sys.argv[1]
    else:
        print(Fore.YELLOW + "Didn't get name for model, exiting")
        sys.exit(1)

    train_model(model_name)

if __name__ == "__main__":
    main()
    
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

import sys
from colorama import Fore, Style, init
import os

init(autoreset=True)

model_name = "model"

if len(sys.argv) > 1:
    model_name = sys.argv[1]
else:
    print(Fore.YELLOW + "Didn't get name for model, model's name set to \"model\"")


model_path = os.path.expanduser(f"~/ALSAI/AI/models/{model_name}")

scan_length = 640

if not os.path.exists(model_path):
    model = Sequential([
        Input(shape=(scan_length, 2)), 
        Conv1D(32, kernel_size=5, activation='relu'),
        Conv1D(64, kernel_size=5, activation='relu'),
        Flatten(),
        Dense(64, activation ='relu'),
        Dropout(0.2),
        Dense(3, activation='linear')
    ])
    model.save(os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.keras"))

for name in os.listdir(os.path.expanduser(f"~/ALSAI/rosbags")):
    if name not in ["map5_run1", "map6_run1"]:
        rosbag_path = os.path.expanduser(f"~/ALSAI/rosbags/{name}")
        train_network.train(rosbag_path, os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.keras"))

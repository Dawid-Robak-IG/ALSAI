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

scan_length = X.shape[1]

model = Sequential([
    Input(shape=(scan_length, 2)), 
    Conv1D(32, kernel_size=5, activation='relu'),
    Conv1D(64, kernel_size=5, activation='relu'),
    Flatten(),
    Dense(64, activation ='relu'),
    Dropout(0.2),
    Dense(3, activation='linear')
])

model.compile(optimizer='adam', loss='mse')
model.summary()


X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2 #, random_state=42
)
print(Fore.GREEN + "Train shape:", X_train.shape, y_train.shape)
print(Fore.GREEN + "Test shape:", X_test.shape, y_test.shape)

early_stop = EarlyStopping(monitor="val_loss", patience=3, restore_best_weights=True)

history = model.fit(
    X_train,
    y_train,
    batch_size=32,
    epochs=50,
    validation_split=0.2,
    callbacks=[early_stop]
)
test_loss = model.evaluate(X_test,  y_test, verbose=0)
print(Fore.GREEN +  "\nTest accuracy:", test_loss)

plt.figure(figsize=(12,4))
plt.plot(history.history['loss'], label='train loss')
plt.plot(history.history['val_loss'], label='val loss')
plt.xlabel('Epoka')
plt.ylabel('Błąd (MSE)')
plt.title('Proces uczenia')
plt.legend()
plt.grid(True)
plt.show()


y_pred = model.predict(X_test)

errors = y_test - y_pred
err_x = errors[:,0]
err_y = errors[:,1]
err_theta = errors[:,2]

plt.figure(figsize=(12,4))
plt.subplot(1,3,1)
plt.hist(err_x, bins=50)
plt.title("Błąd Δx")

plt.subplot(1,3,2)
plt.hist(err_y, bins=50)
plt.title("Błąd Δy")

plt.subplot(1,3,3)
plt.hist(err_theta, bins=50)
plt.title("Błąd Δθ")

plt.show()

# ALSAI

## How to gather data

In order to create gather data you need to put commends:
```bash
- ros2 launch robot_control display.launch.py
- ros2 launch robot_control slam.launch.py
- ros2 launch nav2_bringup navigation_launch.py 
```

and then start listening to data:
```bash
ros2 bag record /odom /scan /joint_states /robot_pose -o ~/ALSAI/rosbags/{name_for_rosbag}
```

## Creating map for gazebo
It's possible to create map for gazebo with:

```bash
python3 svg2sdf.py
```

If your ``svg`` file doesn't have just lines just put it through:

```bash
python3 paths2lines.py
```

## Creating data from rosbag
In order to create data npz based on rosbag run:
```bash
python3 create_data_OnRosbag.py {mapX_runX}
```
and then create test,train data run:
```bash
python3 create_test_train_data.py
```

## Creating model / Training
In order to create model run 
```bash
python3 train_model {model_name}
```
but remember to put name that is in ``build_model.py``

Training will be done on every rosbag in ``ALSAI`` folder but for ``["map5_run1", "map6_run1"]`` (look file ``create_tetst_train_data.py``)

## Testing
In order to test model run 
```bash
python3 test_model {model_name}
```
Testing will give 1 figure. For map **map5_run1** and **map6_run1**.

If you want to test tflite model run:
```bash
python3 test_model {model_name} tflite
```

If you want to test on real data from jetbot run:
```bash
python3 test_model {model_name} jetbot {processing scans}
```
in processing scans wrtie linear for linear interpolation or pick for picking closest point.

## Converting to TF lite
In order to convert model .keras to .tflite run:
```bash
python3 convert_to_tflite.py {model_name}
```

## Graphical model (visual keras)
In order to generate graphical model run:
```bash
python3 graphical_model.py {model_name}
```
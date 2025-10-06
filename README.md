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

## Creating model / Training
In order to create model run 
```bash
python3 train_net_by_rosbags {model_name}
```
but remember to put name that is in ``build_model.py``

Training will be done on every rosbag in ``ALSAI`` folder but for ``["map5_run1", "map6_run1"]`` (look file ``train_net_by_rosbags.py``)

## Testing
In order to test model run 
```bash
python3 test_net_by_rosbags {model_name}
```
Testing will give 2 figures. One for map **map5_run1** and one for **map6_run1** (look file ``test_net_by_rosbags.py``). Second figure will appear after closing first figure.
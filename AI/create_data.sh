#!/bin/bash
python3 create_data_OnRosbag.py map1_run1
python3 create_data_OnRosbag.py map2_run1
python3 create_data_OnRosbag.py map3_run1
python3 create_data_OnRosbag.py map4_run1
python3 create_data_OnRosbag.py map5_run1
python3 create_data_OnRosbag.py map6_run1
python3 create_data_OnRosbag.py map7_run1
python3 create_test_train_data.py

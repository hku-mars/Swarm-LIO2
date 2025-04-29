#!/bin/bash

rosbag record /quad0_pcl_render_node/sensor_cloud /quad_0/imu /quad_0/lidar_slam/odom /quad0_odom_visualization/path /quad1_pcl_render_node/sensor_cloud /quad_1/imu /quad_1/lidar_slam/odom /quad1_odom_visualization/path /quad2_pcl_render_node/sensor_cloud /quad_2/imu /quad_2/lidar_slam/odom /quad2_odom_visualization/path /quadstate_to_teammate


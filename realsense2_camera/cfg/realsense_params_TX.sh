#!/bin/bash

rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_enable_auto_exposure 0
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_exposure 2500

# Set the depth visual preset to HIGH_ACCURACY mode for all cameras
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_laser_power 0
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_emitter_enabled 0

# Setup harware sync
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_output_trigger_enabled 0
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_frames_queue_size 1

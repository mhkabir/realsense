#!/bin/bash

rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_enable_auto_exposure 1

# Set the depth visual preset to HIGH_ACCURACY mode for all cameras
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_emitter_enabled 0

# Setup harware sync
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera_manager rs435_depth_frames_queue_size 1

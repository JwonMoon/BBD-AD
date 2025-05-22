#!/bin/bash

if [ "$1" == "compressed" ]; then
  ros2 bag record /image/compressed /image_left/compressed /image_right/compressed \
	  /carla/hero/GPS /carla/hero/IMU /carla/hero/vehicle_status \
	  /tcp/vehicle_control_cmd /carla/hero/global_plan \
	  -o compressed_bag
else
  ros2 bag record /carla/hero/CAM_FRONT/image /carla/hero/CAM_FRONT_LEFT/image \
	  /carla/hero/CAM_FRONT_RIGHT/image \
	  /carla/hero/GPS /carla/hero/IMU /carla/hero/vehicle_status \
	  /tcp/vehicle_control_cmd /carla/hero/global_plan\
	  -o raw_bag
fi


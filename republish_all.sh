#!/bin/bash

ros2 run image_transport republish raw compressed \
  --ros-args \
    --remap in:=/carla/hero/CAM_FRONT/image \
    --remap out/compressed:=/image_front/compressed \
    --remap __node:=republish_front &

ros2 run image_transport republish raw compressed \
  --ros-args \
    --remap in:=/carla/hero/CAM_FRONT_LEFT/image \
    --remap out/compressed:=/image_front_left/compressed \
    --remap __node:=republish_front_left &

ros2 run image_transport republish raw compressed \
  --ros-args \
    --remap in:=/carla/hero/CAM_FRONT_RIGHT/image \
    --remap out/compressed:=/image_front_right/compressed \
    --remap __node:=republish_front_right &

ros2 run image_transport republish raw compressed \
  --ros-args \
    --remap in:=/carla/hero/CAM_BACK/image \
    --remap out/compressed:=/image_back/compressed \
    --remap __node:=republish_back_front &

ros2 run image_transport republish raw compressed \
  --ros-args \
    --remap in:=/carla/hero/CAM_BACK_LEFT/image \
    --remap out/compressed:=/image_back_left/compressed \
    --remap __node:=republish_back_left &

ros2 run image_transport republish raw compressed \
  --ros-args \
    --remap in:=/carla/hero/CAM_BACK_RIGHT/image \
    --remap out/compressed:=/image_back_right/compressed \
    --remap __node:=republish_back_right &

wait


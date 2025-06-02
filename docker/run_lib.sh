#!/bin/bash

if [ "$#" != "1" ]; then
           echo "Usage: $0 [image_tag]"
           exit 1
fi

TAG=$1
IMAGE=b2d:$TAG

RUNTIME="--runtime=nvidia"

USER_ID=1000

echo "Launching $IMAGE"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

SHARED_DOCKER_DIR=/root/shared_dir/B2D_Demo
SHARED_HOST_DIR=$HOME/ssd_ws/B2D_Demo
#SHARED_DOCKER_CARLA_DIR=/root/shared_dir/carla
#SHARED_HOST_CARLA_DIR=$HOME/orin/ssd_ws/carla/


#mkdir -p $SHARED_HOST_DIR

VOLUMES="--volume=$XSOCK:$XSOCK:rw
        --volume=$XAUTH:$XAUTH:rw
        --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
	--volume=/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu"

xhost +local:docker

docker run \
    -it --rm\
    $VOLUMES \
    $RUNTIME \
    -e DISPLAY=$DISPLAY \
    --env="XAUTHORITY=${XAUTH}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --gpus all \
    --net=host \
    -w /root \
    $IMAGE
#--env="DISPLAY=unix${DISPLAY}" \
# -w /root \
# --user $USER_ID \
# $DEVICES

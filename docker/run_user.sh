#!/bin/bash

if [ "$#" != "1" ]; then
           echo "Usage: $0 [image_tag]"
           exit 1
fi

TAG=$1
IMAGE=uniad:$TAG

RUNTIME="--runtime=nvidia"

USER_ID=1000

echo "Launching $IMAGE"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

SHARED_DOCKER_DIR=/home/misys/shared_dir/B2D
SHARED_HOST_DIR=$HOME/misys/MM/E2E/B2D
SHARED_DOCKER_CARLA_DIR=/home/misys/shared_dir/CARLA_0.9.15
SHARED_HOST_CARLA_DIR=$HOME/misys/carla/CARLA_0.9.15

#mkdir -p $SHARED_HOST_DIR

VOLUMES="--volume=$XSOCK:$XSOCK:rw
        --volume=$XAUTH:$XAUTH:rw
        --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
	--volume=$SHARED_HOST_CARLA_DIR:$SHARED_DOCKER_CARLA_DIR:rw"

xhost +local:docker

docker run \
    -it --rm\
    --user $USER_ID \
    $VOLUMES \
    $RUNTIME \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=unix${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --gpus all \
    --net=host \
    -w /home/misys \
    $IMAGE 
# --user $USER_ID \
# $DEVICES

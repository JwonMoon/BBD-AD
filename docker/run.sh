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

SHARED_DOCKER_DIR=/root/shared_dir/B2D_Demo
SHARED_HOST_DIR=$HOME/misys/MM/B2D_Demo
SHARED_DOCKER_CARLA_DIR=/root/shared_dir/carla
SHARED_HOST_CARLA_DIR=$HOME/misys/carla/


#mkdir -p $SHARED_HOST_DIR

VOLUMES="--volume=$XSOCK:$XSOCK:rw
        --volume=$XAUTH:$XAUTH:rw
        --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
	--volume=$SHARED_HOST_CARLA_DIR:$SHARED_DOCKER_CARLA_DIR:rw
	--volume=/dev/shm:/dev/shm"


xhost +local:docker

docker run \
    -it --rm\
    $VOLUMES \
    $RUNTIME \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=unix${DISPLAY}" \
    --env="UID=$UID" \
    --privileged \
    --gpus all \
    --net host \
    --ipc host \
    --pid host \
     --workdir="$SHARED_DOCKER_DIR" \
    $IMAGE
#    --net host \
# --user $USER_ID \
# $DEVICES

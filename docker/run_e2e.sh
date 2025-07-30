#!/bin/bash

#if [ "$#" != "1" ]; then
#           echo "Usage: $0 [image_tag]"
#           exit 1
#fi

#TAG=$1
#IMAGE=bbd:$TAG
IMAGE=bbd:e2e

RUNTIME="--runtime=nvidia"

USER_ID=1000

echo "Launching $IMAGE"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

SHARED_DOCKER_DIR=/root/shared_dir/BBD-AD
SHARED_HOST_DIR=$HOME/ssd_ws/BBD-AD

#mkdir -p $SHARED_HOST_DIR

VOLUMES="--volume=$XSOCK:$XSOCK:rw
        --volume=$XAUTH:$XAUTH:rw
        --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
	--volume=/dev/shm:/dev/shm"

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
    --workdir="$SHARED_DOCKER_DIR" \
    $IMAGE
# $DEVICES

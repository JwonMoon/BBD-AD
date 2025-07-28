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

SHARED_DOCKER_DIR=/root/shared_dir
SHARED_DOCKER_DIR_B2D=$SHARED_DOCKER_DIR/B2D_Demo

SHARED_HOST_DIR_B2D=$HOME/ssd_ws/B2D_Demo

#mkdir -p $SHARED_HOST_DIR

VOLUMES="--volume=$XSOCK:$XSOCK:rw
        --volume=$XAUTH:$XAUTH:rw
        --volume=$SHARED_HOST_DIR_B2D:$SHARED_DOCKER_DIR_B2D:rw"

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
    --workdir="$SHARED_DOCKER_DIR_B2D" \
    $IMAGE
#--env="DISPLAY=unix${DISPLAY}" \
# -w /root \
# --user $USER_ID \
# $DEVICES

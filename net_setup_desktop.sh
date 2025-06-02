#!/bin/bash

sudo ip addr add 192.168.10.1/24 dev eno1
sudo ip addr add 192.168.30.2/24 dev enx00e04c592990
echo "[Orin2 Setup] Network interfaces configured successfully."



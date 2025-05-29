#!/bin/bash

ip addr add 192.168.10.1/24 dev eno1
ip addr add 192.168.30.2/24 dev enx00e04c592990
echo "[Orin2 Setup] Network interfaces configured successfully."



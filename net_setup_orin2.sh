#!/bin/bash

ip addr add 192.168.20.2/24 dev eno1
ip addr add 192.168.30.1/24 dev enx00e04c5a7590
echo "[Orin2 Setup] Network interfaces configured successfully."


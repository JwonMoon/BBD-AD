#!/bin/bash

# == Modify to suit your environment == 
sudo ip addr add 192.168.10.2/24 dev eth0
sudo ip addr add 192.168.20.1/24 dev eth2
echo "[Orin1 Setup] Network interfaces configured successfully."

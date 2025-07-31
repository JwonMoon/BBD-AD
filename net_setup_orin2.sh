#!/bin/bash

# == Modify to suit your environment == 
sudo ip addr add 192.168.20.2/24 dev eth1
sudo ip addr add 192.168.30.1/24 dev eth2
echo "[Orin2 Setup] Network interfaces configured successfully."

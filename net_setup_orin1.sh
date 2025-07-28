#!/bin/bash

# == Modify to suit your environment == 
sudo ip addr add 192.168.10.2/24 dev eno1
sudo ip addr add 192.168.20.1/24 dev enx1c860b226ad6
echo "[Orin1 Setup] Network interfaces configured successfully."

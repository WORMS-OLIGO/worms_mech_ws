#!/bin/bash

# Define server login information and commands
declare -a servers=("worm1@192.168.42.101" "raspi2@192.168.42.102" "worm3@192.168.42.103" "tomascr@192.168.42.104" "raspi5@192.168.42.105" "worm6@192.168.42.106")

# Define the command to run on each server
command="cd ~/worms_mech_ws"

# Loop through the servers and open a new terminal for each SSH session
for i in "${!servers[@]}"; do
  xterm -e ssh -t "${servers[$i]}" "${commands[$i]}" &
done
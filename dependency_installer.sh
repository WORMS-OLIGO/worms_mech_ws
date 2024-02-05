#!/bin/bash

# Ensure the script is run with superuser privileges
if [[ $EUID -ne 0 ]]; then
   echo "Please run this script as root or use sudo"
   exit 1
fi

# Update
apt update 

# Silent install for figlet and toilet
dpkg -l | grep -qw toilet || apt-get install -y toilet > /dev/null 2>&1

# ASCII Art Headers
echo -e "\e[31m"
toilet -f smblock -w 150 "WORMS Dependency Installer"

# List of libraries to install
libraries=(
    "ros-humble-joint-state-publisher-gui"
    "pip install serial"
    # Add more libraries below this line
)

# Progress bar function
progress_bar() {
    local total=$1
    local current=$2

    local percentage=$((100*current/total))
    local progress=$((percentage/2))
    local remainder=$((50-progress))

    echo -n "["
    printf "#%.0s" $(seq 1 $progress)
    printf " %.0s" $(seq 1 $remainder)
    echo -n "] $percentage% Installed"
}

total=${#libraries[@]}

# Loop through the libraries and install each one
for i in "${!libraries[@]}"; do
    lib="${libraries[$i]}"
    echo -e "\e[34m[INFO]\e[0m Installing $lib..."
    apt install "$lib" -y > /dev/null 2>&1 || {
        echo -e "\e[31m[ERROR]\e[0m Failed to install $lib. Exiting."
        exit 1
    }
    progress_bar $total $(($i + 1))
    echo ""
done

echo -e "\e[32m[SUCCESS]\e[0m All libraries have been installed!"

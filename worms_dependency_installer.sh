#!/bin/bash

# Ensure the script is run with superuser privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root or with sudo."
   exit 1
fi

echo "Wrangling the WORMS - System Initializing..."

# Check for toilet and install if necessary

echo "Installing text display package..."
apt-get install -y toilet > /dev/null 2>&1
echo "Text display installation complete."


# Display ASCII Art Header
echo -e "\e[31m"
toilet -f smblock -w 150 "WORMS Dependency Installer"

echo "Updating Debian/APT Packages"

# Update package lists
apt update 

echo "Installing Joy:"
apt-get install ros-humble-joy-node

echo "Installing pip:"
# Ensure pip is installed
apt-get install -y python3-pip 

echo "Installing python libraries:"
pip install serial
pip install pandas
pip install pyzbar
pip install opencv-python


echo "Installing motor library"
cd src/worms_mech/mini-cheetah-tmotor-python-can
pip install .

cd 

echo -e "\e[32m[SUCCESS]\e[0m All libraries have been installed!"

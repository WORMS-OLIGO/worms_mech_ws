#!/bin/bash

# Ensure the script is run with superuser privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root or with sudo."
   exit 1
fi

echo "Wrangling the WORMS - System Initializing..."

# Check for toilet and install if necessary
if dpkg -l | grep -qw toilet; then
    echo "Text display package is already installed, moving on..."
else
    echo "Installing text display package..."
    apt-get install -y toilet > /dev/null 2>&1
    echo "Text display installation complete."
fi

# Display ASCII Art Header
echo -e "\e[31m"
toilet -f smblock -w 150 "WORMS Dependency Installer"

# Update package lists
apt update 

# System libraries to install with apt-get
system_libraries=(
    "curl"
    "git"
    # Add more system packages here
)

# Python libraries to install with pip
python_libraries=(
    "serial"
    # Add more Python libraries here
)

# Install system libraries
for lib in "${system_libraries[@]}"; do
    echo -e "\e[34m[INFO]\e[0m Installing system library: $lib..."
    apt-get install -y $lib > /dev/null 2>&1 || {
        echo -e "\e[31m[ERROR]\e[0m Failed to install $lib. Exiting."
        exit 1
    }
done

# Ensure pip is installed
apt-get install -y python3-pip > /dev/null 2>&1

# Install Python libraries
for lib in "${python_libraries[@]}"; do
    echo -e "\e[34m[INFO]\e[0m Installing Python library: $lib..."
    pip3 install $lib > /dev/null 2>&1 || {
        echo -e "\e[31m[ERROR]\e[0m Failed to install $lib. Exiting."
        exit 1
    }
done

echo -e "\e[32m[SUCCESS]\e[0m All libraries have been installed!"

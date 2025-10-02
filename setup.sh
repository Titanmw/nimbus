#!/bin/bash

# Get the current working directory
CURRENT_PATH=$(pwd)

# Clone the PX4 Firmware repository
if [ ! -d "$CURRENT_PATH/PX4-Autopilot" ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git
else
    echo "PX4-Autopilot directory already exists, skipping clone."
fi

# Setup the PX4 development environment (this script installs all necessary dependencies)
if [ -f "$CURRENT_PATH/PX4-Autopilot/Tools/setup/ubuntu.sh" ]; then
    # Ensure the script has execute permissions
    chmod +x "$CURRENT_PATH/PX4-Autopilot/Tools/setup/ubuntu.sh"
    
    # Run the setup script with the full path
    bash "$CURRENT_PATH/PX4-Autopilot/Tools/setup/ubuntu.sh"
else
    echo "ubuntu.sh script not found!"
fi

# Optionally flush DNS cache if necessary (uncomment if needed)
# sudo systemd-resolve --flush-caches

# Run the jMAVSim simulation
if [ -d "$CURRENT_PATH/PX4-Autopilot" ]; then
    make -C "$CURRENT_PATH/PX4-Autopilot" px4_sitl_default jmavsim
else
    echo "PX4-Autopilot directory does not exist!"
fi


#falls fatal: unable to access 'https://github.com/PX4/heatshrink.git/': Could not resolve host: github.com
#   dann:
#       sudo nano /etc/resolv.conf
#       nameserver 8.8.8.8
#       nameserver 8.8.4.4

#Python lib
pip install mavsdk

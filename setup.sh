# Clone the PX4 Firmware repository
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Setup the PX4 development environment (this script installs all necessary dependencies)
bash ./Tools/setup/ubuntu.sh

sudo systemd-resolve --flush-caches

# Run jMAVSim simulation
make px4_sitl jmavsim
#falls fatal: unable to access 'https://github.com/PX4/heatshrink.git/': Could not resolve host: github.com
#   dann:
#       sudo nano /etc/resolv.conf
#       nameserver 8.8.8.8
#       nameserver 8.8.4.4

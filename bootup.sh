sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
source /opt/ros/humble/setup.bash
cd ~/worms_mech_ws/
source install/setup.bash
ros2 daemon stop 
ros2 daemon start
sudo python3 camera_looper.py

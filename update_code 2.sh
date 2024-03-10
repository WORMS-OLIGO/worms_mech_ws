git reset --hard
git pull
rm -r install
rm -r build
colcon build
source install/setup.bash

# holiday ros2
cd ${HOLIDAY_ROS_ROOT}/src
#git clone -b main git@github.com:Holiday-Robot/holiday-robotics-library.git
git clone -b main git@github.com:Holiday-Robot/holiday-ros2-interface.git

cd ${HOLIDAY_ROS_ROOT}
colcon build --symlink-install
echo "
# Holiday ROS2 environment
source /holiday/ros2/install/setup.zsh" >> ~/.zshrc

nvidia-smi

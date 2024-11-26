AGIMUS demo 00 franka controller
--------------------------------

## Start the demo in simulation using the Panda robot.
The purpose of this demo is to simply check whether your installation is done correctly.
To verify it is correctly working, after launching the bringup launch file, a gazebo and a rviz simulations should be appearing with the panda in it.

cd src
git clone -b humble_devel git@github.com:agimus-project/franka_description.git
cd ..
source install/setup.bash
colcon build
ros2 launch agimus_demo_00_franka_controller bringup_gz_simulation.launch.py
AGIMUS demo 02 simple pd plus for TIAGo Pro
-------------------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
The LFC input is comming from a python test node called `pd_plus_controller`. It fills the control object to design a PD+ controller tracking a small sinus signal at the joint level.
Expected behavior: robot's joints are oscillating gently.
The parameters for the motion is located in [agimus_demo_02_simple_pd_plus_tiago_pro/config/pd_plus_controller.yaml](config/pd_plus_controller.yaml).

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_02_simple_pd_plus_tiago_pro/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the TIAGo Pro robot.
```bash
cd workspace
reset && source install/setup.bash && ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py is_public_sim:=True
```

If you are in a docker provided by PAL robotics with an Alum distribution you can run:
```bash
cd workspace
reset && source install/setup.bash && ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py
```

After the simulation to launch the controllers you would need to run the controllers:
```bash
ros2 launch tiago_pro_controller_configuration agimus_controller.launch.py```
```

And activate them for the left arm:
```bash
ros2 control switch_controllers --deactivate arm_left_controller --activate joint_state_estimator linear_feedback_controller
```

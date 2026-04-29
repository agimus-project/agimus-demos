# Demo 07 Fixed — Real robot procedure

> [!CAUTION]
> Before starting, make sure the robot is in a safe position with sufficient
> movement space for the right arm. Ensure all spectators are at a safe distance
> and **the operator can quickly reach the Emergency Button**.

---

## 1. Install the linear-feedback-controller on the robot

- Get an alum docker which is a mirror of the robot OS.
- Create a workspace with the following 2 packages:
  - https://github.com/loco-3d/linear-feedback-controller
  - https://github.com/loco-3d/linear-feedback-controller-msgs
- Use the pal deploy tooling to install this workspace onto the robot.
  See the official PAL documentation.

---

## 2. Setup the environment (in the dev container)

Install cyclonedds and mcap plugin:
```bash
sudo apt update
sudo apt install ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap
```

Clone and build the workspace:
```bash
cd ros2_ws/src
git clone git@github.com/agimus-project/agimus-demos
git clone git@github.com/agimus-project/agimus_controller
vcs-import agimus-demos/tiago_pro.repos
```

Change the network interface in `agimus-demos/agimus_demos_common/config/tiago_pro/cyclone_config.xml`
with your own.

Copy LFC + JSE config to `/tmp` in the docker and on the robot:
```bash
cp agimus-demos/agimus_demos_common/config/tiago_pro/* /tmp/
scp agimus-demos/agimus_demos_common/config/tiago_pro/* pal@tiago-pro:/tmp/
```

Configure cycloneDDS in your bashrc:
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export CYCLONEDDS_URI=/tmp/cyclone_config.xml" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
```

Remove the laser from the robot URDF:
```bash
pal robot_info set laser_model no-laser
```

Build the workspace:
```bash
cd ~/ros2_ws
./setup.sh
./build.sh
```

---

## 3. Localize the pylone

Before running the demo, the pylone pose relative to the robot must be estimated.
Place the pylone in its intended position (no table needed — the pylone can be on any support). Then run the localization script:

```bash
python3 ros2_ws/install/agimus_demo_07_fixed_tiago_pro_deburring/share/agimus_demo_07_fixed_tiago_pro_deburring/scripts/localize_pylone.py
```

This will:
1. Switch the right arm to gravity compensation — you can move it freely.
2. Ask you to point the gripper tip at 3 holes of the pylone (press Enter to record each).
3. Compute the pylone pose and print a YAML snippet to copy into `hpp_orchestrator_params.yaml`.

To use specific holes (default: `hole_right_00`, `hole_right_25`, `hole_right_49`):
```bash
python3 localize_pylone.py --holes hole_right_10 hole_right_30 hole_right_49
```

The result is automatically written to `config/pylone_pose.yaml`.

---

## 4. Launch the demo

Restart the controllers on the robot:
```bash
pal module_manager restart
```

Launch the bringup (with `ssh -X` for remote connection):
```bash
ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py use_rviz:=true
```

---

## 5. Launch the HPP orchestrator

In a separate terminal:

```bash
python3 ~/ros2_ws/install/agimus_demo_07_fixed_tiago_pro_deburring/share/agimus_demo_07_fixed_tiago_pro_deburring/hpp/orchestrator_node.py
```

---

## 6. Run the demo (in the IPython shell)

```python
# Verify pylone pose in Viser before moving
o.init_viewer()
o.reload_pylone_pose()   # reads pose from hpp_orchestrator_params.yaml

# Sync arm position from robot
o.sync_from_robot()

# When ready (emergency stop in hand): switch to torque control
o.activate_lfc()

# Plan and execute
o.plan()
o.execute()

# Or both at once
o.plan_and_execute()
```

Once `activate_lfc()` is called, the arm switches from position to torque control.
The MPC will track the planned trajectory. Use the emergency stop if needed.

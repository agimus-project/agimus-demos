AGIMUS demo 10 bi-manipulation with TIAGo Pro
--------------------------------------------------

The purpose of this demo is to implement the Agimus case study CS 2.1 (Kleeman).

Tiago pro manipulates a reinforcment bar with two hands in order to glue it to
an elevator ceiling.

## Dependencies

This demo requires source built of dependencies found in:
- [control.repos](../control.repos)
- [tiago-pro.repos] (../tiago-pro.repos)

## Simulation

### Launch Simulation
```bash
ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py \
                is_public_sim:=True \
                world_name:=empty \
                end_effector_right:=pal-pro-gripper \
                end_effector_left:=pal-pro-gripper \
                tuck_arm:=False \
                gazebo_version:=gazebo \
                gzclient:=False
```
### Launch HPP orchestrator
```bash
ros2 launch agimus_demo_10_tiago_pro_bar_manip bringup.launch.py use_sim_time:=True
```
### Request Grasping
```bash
ros2 action send_goal /orchestrator/plan_bar_handling test_agimus_type/action/PlanBarGrasp "{action_type: 'grasp', gripper: 'tiago_pro/left', handle: 'reinforcement_bar/left'}"
```
### Request Placement
```bash
ros2 action send_goal /orchestrator/plan_bar_handling test_agimus_type/action/PlanBarGrasp "{action_type: 'place', gripper: 'tiago_pro/left', handle: 'reinforcement_bar/left'}"
```
## Real robot

WIP

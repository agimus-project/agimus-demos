# Demo 07 Fixed — Real robot procedure

> [!CAUTION]
> Before starting, make sure the robot is in a safe position with sufficient
> movement space for the right arm. Ensure all spectators are at a safe distance
> and **the operator can quickly reach the Emergency Button**.

---

## 1. Install the linear-feedback-controller on the robot

- Get an alum docker which is a mirror of the robot OS.
- run sudo apt install pal-alum-linear-feedback-controller
- Use the pal deploy tooling to install this workspace onto the robot.
  See the official PAL documentation.

---

## 2. Setup the environment (in the dev container)

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

On the robot, restart controllers
```bash
pal module restart controller_manager

# if no controller running, launch
pal module start default_controllers
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
Place the pylone in its intended position (no table needed — the pylone can be on any support).

### Option A — Manual pointing (no mocap required)

Run the localization script:

```bash
python3 ros2_ws/install/agimus_demo_07_fixed_tiago_pro_deburring/share/agimus_demo_07_fixed_tiago_pro_deburring/scripts/localize_pylone.py
```

This will:
1. Switch the right arm to gravity compensation — you can move it freely.
2. Ask you to point the gripper tip at 3 holes of the pylone (press Enter to record each).
3. Compute the pylone pose and print a YAML snippet to copy into `hpp_orchestrator_params.yaml`.

To use specific holes (default: `hole_right_00`, `hole_right_50`, `hole_right_59`):
```bash
python3 localize_pylone.py --holes hole_right_10 hole_right_30 hole_right_49
```

The result is automatically written to `config/pylone_pose.yaml`.

### Option B — Mocap-based localization (Qualisys required)

Once the mocap is connected (see [section 5.5](#55-mocap-qualisys-optional)), run in the IPython shell:

```python
o.localize_pylone_from_mocap()   # reads pylone pose from mocap, saves to pylone_pose.yaml
```

---

## 4. Launch the demo

Restart the controllers on the robot:
```bash
pal module_manager restart
```

Launch the bringup (with `ssh -X` for remote connection):
```bash
# Without mocap (default)
ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py use_rviz:=true

# With mocap — publishes mocap_ee TF and corrects MPC targets in real time
ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py \
    use_rviz:=true use_mocap:=true
```

With `use_mocap:=true`, two additional nodes are started automatically:
- **`mocap_ee_publisher`** — connects to Qualisys and publishes the EE pose as a TF frame (`base_link → mocap_ee`) and on `/mocap_ee_pose`
- **`mocap_mpc_corrector`** — intercepts `mpc_input`, computes the translation error `δt = t_FK − t_mocap` at the current configuration, adds it to every target EE pose, and republishes on `mpc_input_corrected` (which the controller then consumes)

---

## 5. Launch the HPP orchestrator

In a separate terminal:

```bash
python3 src/agimus-demos/agimus_demo_07_fixed_tiago_pro_deburring/hpp/orchestrator_node.py
```

---

## 5.5 Mocap (Qualisys)

The Qualisys server IP is `140.93.1.100` by default.

### Verify body IDs before first use

QTM body IDs must match the values set in `_MOCAP_BODIES` across all scripts and the orchestrator.
Verify with:

```bash
python3 scripts/qualisys.py discover
# or with a custom IP:
python3 scripts/qualisys.py discover 192.168.x.x
```

Expected mapping:

| QTM ID | Name |
|---|---|
| 0 | `pylone` |
| 1 | `tiago_base` |
| 2 | `tiago_endEffector` |

If IDs differ, update `_MOCAP_BODIES` consistently in **all three files** (keep key order, update values only):
- `hpp/orchestrator.py`
- `scripts/mocap_ee_publisher.py`
- `scripts/calibrate_mocap_fk.py`

### MPC correction (via launch)

When `use_mocap:=true` is passed to the launch file, mocap-based MPC correction is fully
automatic — no extra steps needed. In RViz you can verify the `mocap_ee` frame aligns with
`gripper_right_tool_holder`.

### Orchestrator utilities (IPython shell)

These tools are available independently of the launch-level correction:

```python
o.connect_mocap()              # connect to Qualisys (required for all tools below)
o.connect_mocap("192.168.x.x") # or with a custom IP
o.disconnect_mocap()           # stop the Qualisys subprocess
```

**Visualize mocap frames in Viser** — two live coordinate-axis frames:
- `mocap/ee` — `tiago_endEffector` relative to `tiago_base`
- `mocap/pylone` — `pylone` relative to `tiago_base`

```python
# One-shot update
o.update_mocap_frames()

# Live loop
import time
while True:
    o.update_mocap_frames()
    time.sleep(0.1)
```

Requires `connect_mocap()` and `init_viewer()` to be called first.

**Compare mocap vs robot FK (numerical):**

```python
o.compare_mocap()  # prints per-axis position error (mm) and rotation error (°) for EE and pylone
```

Output example:
```
==================================================================
  Mocap vs Robot — poses relative to base_footprint / tiago_base
==================================================================

  End effector  (tiago_endEffector ↔ gripper_right_tool_holder)
                       x             y             z
      mocap [m]   +0.4123       -0.1234       +0.7456
      robot [m]   +0.4101       -0.1219       +0.7448
      Δ [mm]       +2.20         -1.50         -0.80   (|Δ|=2.8 mm)  ✓
      ...
```

**Analyse FK vs mocap consistency** (calibration check):

```python
# Record pairs (T_FK, T_mocap) at multiple arm configurations
c = Calibrator()    # see scripts/calibrate_mocap_fk.py
c.connect_mocap()
# move arm, then:
c.record()          # repeat 10–15 times at different configs
c.analyze()         # prints mean error and config-to-config dispersion
```

If `σ_t < 3 mm` and `σ_R < 1°`, the error is a constant FK offset and the MPC correction is reliable.

---

## 6. Run the demo (in the IPython shell)

```python
# Verify pylone pose in Viser before moving
o.init_viewer()
o.reload_pylone_pose()   # reads pose from pylone_pose.yaml

# Sync arm position from robot
o.sync_from_robot()

# When ready (emergency stop in hand): switch to torque control
o.activate_lfc()

# Plan
o.plan()

# Execute step by step (recommended on real robot)
o.execute([o.p1])        # approach only — check before going further
o.compare_pose(o.p1)     # verify tracking error before insertion
o.execute([o.p2])        # insertion
o.execute([o.p3])        # retraction
o.execute([o.p4])        # retreat back to q_init

# Or full sequence at once
o.plan_and_execute()     # p1 → p2 → p3 → p4
```

Once `activate_lfc()` is called, the arm switches from position to torque control.
The MPC will track the planned trajectory. Use the emergency stop if needed.

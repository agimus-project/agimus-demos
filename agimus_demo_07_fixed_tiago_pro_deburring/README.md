# Demo 07 — TIAGo Pro Deburring, Fixed Base (Whole Body MPC + HPP)

Deburring demo with TIAGo Pro in fixed-base configuration: the right arm approaches
a hole on the pylone, inserts the tool, then retracts. The mobile base does not move.

Motion planning is handled by **HPP** (Humanoid Path Planner) and execution by
the **agimus_controller** (Whole Body MPC).

---

## Architecture

```
HPP orchestrator (IPython shell, HPP env)
  └─> publishes MpcInput on /mpc_input
        └─> agimus_controller (Whole Body MPC)
              └─> linear_feedback_controller (LFC)
                    └─> joint torque commands → robot / Gazebo
```

**Active DOFs:** right arm only (7)

The left arm, torso, wheels, grippers, head and mobile base are locked via
`LockedJoint` constraints in the HPP model (`anchor` root joint).

### Planning pipeline

```
q_init → [p1: approach] → qpg → [p2: insertion] → qg
                                                    ↓
                          qpg ← [p3: retraction] ←─┘
```

- **p1** (approach): right arm moves to pre-grasp pose in front of the hole
- **p2** (insertion): arm inserts along handle Z (world +Y, into the hole)
- **p3** (retraction): reverse of p2

### Key files

| File | Role |
|---|---|
| `launch/bringup.launch.py` | Main launch file |
| `config/agimus_controller_params.yaml` | MPC node parameters |
| `config/hpp_orchestrator_params.yaml` | Handle, tuck poses, MPC weights |
| `config/pylone_pose.yaml` | Pylone pose in base_link frame (editable per setup) |
| `hpp/orchestrator.py` | HPP planning + MPC publishing class |
| `hpp/orchestrator_node.py` | Interactive IPython entry point |
| `scripts/localize_pylone.py` | Pylone pose estimation from manual pointing |

---

## Dependencies

This demo requires source built of dependencies found in:
- [control.repos](../control.repos)

---

## Simulation

> [!NOTE]
> Gazebo simulation of TIAGo Pro requires high CPU frequency. Older machines may
> experience high load or timing errors.

### Step 1 — Launch Gazebo + MPC controller

```bash
ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py \
    use_gazebo:=true
```

This starts Gazebo with TIAGo Pro, the pylone, the LFC + JSE controllers,
and the agimus_controller waiting for a trajectory.

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `use_rviz` | `false` | Open RViz |
| `use_mpc_debugger` | `false` | Launch MPC debugger |

### Step 2 — Launch the HPP orchestrator

In a separate terminal:

```bash
python3 <install>/agimus_demo_07_fixed_tiago_pro_deburring/hpp/orchestrator_node.py
```

### Step 3 — Plan and execute (in the IPython shell)

```python
o.sync_from_robot()      # sync right arm position from Gazebo
o.activate_lfc()         # switch arm to torque control
o.plan()                 # run HPP planner
o.execute()              # publish trajectory to MPC

# Or all at once (after activate_lfc)
o.plan_and_execute()

# Visualise in Viser
o.init_viewer()
o.play(o.p1)
o.play(o.p2)
o.play(o.p3)
```

---

## Scene geometry

Pylone position is defined in `config/pylone_pose.yaml` and can be edited directly
or overwritten by `scripts/localize_pylone.py` on the real robot.

```
Pylone base (default):  x=1.5, y=0.0, z=1.03 m (base_link frame)
Active hole:            right face, y=-0.213, x=0.0, z=0.0 (pylone frame)
```

---

## Real robot

See [README_real_robot.md](README_real_robot.md).

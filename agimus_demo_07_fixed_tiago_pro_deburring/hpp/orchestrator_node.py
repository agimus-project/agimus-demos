#!/usr/bin/env python3
"""
Entry point for the HPP deburring orchestrator.

Run after sourcing ros2_config.sh, ros2_ws/install/setup.bash, and hpp_config.sh.

Usage:
    python3 orchestrator_node.py

Drops into an IPython shell with the orchestrator pre-loaded:

    o.plan()              # plan with HPP (approach + insert + retract)
    o.execute()           # publish trajectory to MPC controller
    o.plan_and_execute()  # both
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from orchestrator import Orchestrator

rclpy.init()

o = Orchestrator()

banner = (
    "\n"
    "╔══════════════════════════════════════════════════════════╗\n"
    "║   TIAGo Pro — HPP Deburring Orchestrator                 ║\n"
    "╠══════════════════════════════════════════════════════════╣\n"
    "║  o.sync_from_robot()         — sync q_init from robot state    ║\n"
    "║  o.reload_pylone_pose()      — reload pylone pose from config   ║\n"
    "║  o.activate_lfc()            — switch to torque control        ║\n"
    "║  o.plan()                    — run HPP planner                 ║\n"
    "║  o.execute()                 — publish trajectory to MPC       ║\n"
    "║  o.plan_and_execute()        — plan then execute               ║\n"
    "║  o.init_viewer()             — open Viser viewer               ║\n"
    "║  o.view(q)                   — show config in Viser            ║\n"
    "║  o.play(path)                — animate a path in Viser         ║\n"
    "╚══════════════════════════════════════════════════════════╝\n"
)

try:
    import IPython
    IPython.embed(banner1=banner, user_ns={"o": o, "rclpy": rclpy})
except ImportError:
    import code
    code.interact(banner=banner, local={"o": o, "rclpy": rclpy})

if o._ros_node is not None:
    o._ros_node.destroy_node()
rclpy.shutdown()

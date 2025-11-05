# Agimus Demos

Agimus Demos is a set of ROS 2 packages used to launch different demos developed for [Agimus Project](https://www.agimus-project.eu/).

The repository is organized as set of demos with increasing complexity.
Each folder contains a ROS 2 package launching the demo.
The `README.md` file of each demo contains a detailed description of the expected results and instruction on how to launch the demo.

## Table of content

- [Usage](#Usage)
- [Installation](#Installation)

## Usage

### Running demos

All of the demos are launched in a similar manner. For Gazebo simulation use
```bash
ros2 launch agimus_demo_<demo-name> bringup.launch.py use_gazebo:=true use_rviz:=true
```
For the real robot use
```bash
ros2 launch agimus_demo_<demo-name> bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true
```

### Plotting and debugging

> [!CAUTION]
> This is work in progress and subject to change.

The following tools assume that agimus-controller node was started with parameter `publish_debug_data` set to `true`.

You can visualize MPC outputs using PlotJuggler with
```bash
ros2 launch agimus_demos_common plotjuggler_mpc.launch.py
```

You can visualize the current prediction of MPC in RViz in two steps:
```bash
ros2 run agimus_controller_ros mpc_debugger_node --frame fer_hand_tcp
```
Then use Rviz to display the marker array published to `mpc_states_prediction_markers`.
If your RViz config file is up-to-date, the visualization is already configured and just needs to be enabled by checking the box of `MPC predictions`.

## Installation

### Running demos with Docker

> [!NOTE]
> This is a recommended installation for this package. You can always install it from source (see below), but to avoid issues with building from source we advise you to use prebuilt docker images, that contains all the dependencies.

> [!CAUTION]
> Current demos only make use of `agimus_dev_container:humble-devel-control`. There is no need to pull Docker images with vision dependencies.

Docker image is provided in a form of Development Container and can be found at [agimus-project/agimus_dev_container](https://gitlab.laas.fr/agimus-project/agimus_dev_container). Use branch `humble-devel` and refer to the README.md file for more information on how to use it.

This docker has all the dependencies preinstalled and is split into three images: control, vision and vision-cuda. Control image contains dependencies found in [franka.repos](franka.repos) and [control.repos](control.repos). Vision image is not yet supported by this repository.

To run demos in the docker image, first download the repository
```bash
git clone -b humble-devel https://gitlab.laas.fr/agimus-project/agimus_dev_container.git
# Open folder in VS Code
code agimus_dev_container
```
Then following the instruction from [README.md](https://gitlab.laas.fr/agimus-project/agimus_dev_container/-/blob/humble-devel/README.md?ref_type=heads) reopen the folder in Development Container. Once you are inside of the Development Container clone Adimus Demos.

```bash
git clone https://github.com/agimus-project/agimus-demos.git ~/ros2_ws/src/agimus-demos
cd ~/ros2_ws
# Ensure all dependencies are installed and up to date
./setup.sh
# Build your workspace
./build.sh
# Source the workspace
source install/setup.bash
```

Now you are ready to run all of your demos!

### Building dependencies from source

> [!IMPORTANT]
> Not all demos require all dependencies. Check README.md files of each demo to learn which dependencies are required.

> [!TIP]
> Eigenpy and Pinocchio have high memory footprint then built from source. Adjust `-j` flag in `MAKEFLAGS` make sure you will not run out of RAM on your machine. Current setting should work for machines with 16 GB or RAM.

```bash
cd ~/ros2_ws
git clone https://github.com/agimus-project/agimus-demos.git src/agimus-demos

# Clone dependencies required by Franka robots
vcs import --shallow --recursive src < src/agimus-demos/franka.repos
# Clone base dependencies for optimal control
vcs import --shallow --recursive src < src/agimus-demos/control.repos
# Clone Agimus-specific dependencies
vcs import --shallow --recursive src < src/agimus-demos/agimus_dev.repos

# Pinocchio has a hardcoded hpp-fcl as dependency while we expect Coal
# Until it is fixed we need to change it manually
sed -i 's/hpp-fcl/coal/g' src/vcs_control/pinocchio/package.xml

# Clone dependencies for path planning
vcs import --recursive src < src/agimus-demos/planning.repos
# Additional dependencies required by HPP
mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update
sudo apt install-y \
    robotpkg-qt5-osgqt \
    robotpkg-qpoases \
    robotpkg-py310-omniorbpy \
    robotpkg-qt5-qgv


sudo apt update
rosdep update --rosdistro $ROS_DISTRO
# Install all dependencies that are available as binaries
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO

# Source ROS base to make sure all installed packages are discovered
source /opt/ros/$ROS_DISTRO/setup.bash
# Set maximum number of CPU cores used by `make` while building packages
# When building with planning dependencies number of cores has to be set to 1
export MAKEFLAGS="-j 6"
# Build the workspace. Merge install is required to make Python bindings work
colcon build \
    --symlink-install \
    --merge-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_BENCHMARK=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DBUILD_PYTHON_INTERFACE=ON \
        -DGENERATE_PYTHON_STUBS=OFF \
        -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON \
        -DCOAL_HAS_QHULL=ON \
        -DBUILD_WITH_COLLISION_SUPPORT=ON \
        -DBUILD_WITH_MULTITHREADS=ON

# Source the workspace
source install/setup.bash
```

### Dual computer setup

Dual computer setup with real time computer can be used to ensure stable control. Auxiliary computer with real-time kernel is expected to have docker installed and ssh keys exchanged with non-real-time machine. User account on the auxiliary computer requires access to `docker` group and a group with real time priorities.

Before launching the demo, you have to exchange ssh keys:
```bash
# Copy ssh keys to real-time computer to enable ssh connections
ssh-copy-id <remote username>@<remote ip>
```
Once ssh keys are exchanged you can use the following commands to start the controller:
```bash
ros2 launch agimus_demo_<demo-name> bringup.launch.py robot_ip:=<robot-ip> aux_computer_ip:=<remote ip> aux_computer_user:=<remote username>
```
This will automatically start a docker container with real time controllers on the specified auxiliary computer and launch all remaining nodes on the machine where this command is executed.

> [!NOTE]
> In many cases when ROS launch is stopped, the auxiliary computer leaves docker container running. Then the docker container has to be either manually stopped from terminal, or in case Linear Feedback Controller is running this can be done by pressing emergency stop of the robot. Emergency stop interrupts the controller end interrupts the execution of the docker container.

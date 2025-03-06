# Agimus Demos

Agimus Demos is a set of ROS 2 packages used to launch different demos developed for [Agimus Project](https://www.agimus-project.eu/).

The repository is organized as set of demos with increasing complexity.
Each folder contains a ROS 2 package launching the demo.
The `README.md` file of each demo contains a detailed description of the expected results and instruction on how to launch the demo.

All of the demos are launched in a similar manner. For Gazebo simulation use
```bash
ros2 launch agimus_demo_<demo-name> bringup.launch.py use_gazebo:=true use_rviz:=true
```
For the real robot use
```bash
ros2 launch agimus_demo_<demo-name> bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true
```

## Running demos with Docker

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

## Building dependencies from source

> [!IMPORTANT]
> Not all demos require all dependencies. Check README.md files of each demo to learn which dependencies are required.

> [!TIP]
> Eigenpy and Pinocchio have high memory footprint then built from source. Adjust `-j` flag in `MAKEFLAGS` make sure you will not run out of RAM on your machine. Current setting should work for machines with 16 GB or RAM.

```bash
cd ~/ros2_ws
git clone https://github.com/agimus-project/agimus-demos.git src/agimus-demos
# Clone dependencies required by Franka robots
vcs import --recursive src < src/agimus-demos/franka.repos
# Clone dependencies for MPC and collision avoidance
vcs import --recursive src < src/agimus-demos/control.repos
# Pinocchio has a hardcoded hpp-fcl as dependency while we expect Coal
# Until it is fixed we need to change it manually
sed -i 's/hpp-fcl/coal/g' src/vcs_control/pinocchio/package.xml

sudo apt update
rosdep update --rosdistro $ROS_DISTRO
# Install all dependencies that are available as binaries
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO

# Source ROS base to make sure all installed packages are discovered
source /opt/ros/$ROS_DISTRO/setup.bash
# Set maximum number of CPU cores used by `make` while building packages
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

## Dual computer setup
Dual computer setup with real time computer can be used to ensure stable control. Auxiliary computer with real-time kernel is expected to have docker installed and ssh keys exchanged with non-real-time machine.
To exchange ssh keys you can use following command:
```bash
# Copy ssh keys to real-time computer to enable ssh connections
ssh-copy-id <remote username>@<remote ip>
```
Once ssh keys are exchanged you can use following commands to start the controller:
```bash
ros2 launch agimus_demo_<demo-name> bringup.launch.py robot_ip:=<robot-ip> aux_computer_ip:=<remote ip> aux_computer_user:=<remote username>
```
This will automatically start and later a docker container on the remote machine.

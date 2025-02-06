# Agimus Demos

Agimus Demos is a set of ROS 2 packages used to launch different demos developed for [Agimus Project](https://www.agimus-project.eu/).

Repository is organized as set of demos with increasing complexity. Each folder contains a ROS 2 package launching the demo. README.md of each demo contains description with expected results and instruction how to launch the demo.

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

## Building dependencies from source

> [!IMPORTANT]
> Not all demos require all dependencies. Check README.md files of each demo to learn which dependencies are required.

```bash
cd ~/ros2_ws
git clone https://github.com/agimus-project/agimus-demos.git src/agimus-demos
# Clone dependencies required by Franka robots
vcs import --recursive src < src/agimus-demos/franka.repos
# Clone dependencies for MPC and collision avoidance
vcs import --recursive src < src/agimus-demos/control.repos

sudo apt update
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i \
    --from-paths src \
    --rosdistro $ROS_DISTRO \
    --skip-keys libfranka

colcon build \
    --symlink-install \
    --cmake-args \
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


source install/setup.bash
```

> [!TIP]
> If you want full source build you can additionally run
> ```bash
> vcs import src < src/agimus-demos/control.source.repos
> ```
> Before running
> ```bash
> vcs import src < src/agimus-demos/control.repos
> ```
> This way you will install Pinocchio and it's dependencies from source in case you need to set custom build flags.

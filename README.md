# AUT-720 Advanced Robotics
We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments.  

## Prerequisites
It is recommended to use the following requirements to avoid any unforseen error during the implementation of the codes.

- Ubuntu 20.04
- ROS Noetic

## VMWare (Optional)
If you are using windows or run an older version of ubuntu on your PC, you can use VMWare Workstation Player to run ubuntu 20 which is free for non-commercial use. In that case, follow the instructions below to install and prepare you virtual machine:

- depending on your current version of operating system, download one of the installers from [VMWare Installers](https://www.vmware.com/fi/products/workstation-player/workstation-player-evaluation.html).
- on windows, just run the installer GUI and follow the instructions
- on Ubuntu install it from command lines:
```
$ cd "PATH/TO/DOWNLOADED/FOLDER"  #change directory to where you have downloaded the .bundle (installer) file
$ sudo sh VMware-Player-15.5.6-16341506.x86_64.bundle  #change .bundle file name if necessary
```
- Download an ubuntu .iso image from [Ubuntu 20 ISO file](https://releases.ubuntu.com/20.04/ubuntu-20.04.4-desktop-amd64.iso)
- open VMWare Workstation Player and create an new environment from the ISO file and set the hardware properties as you wish.


## Installation
The github repository provided here is the modified version of [4] that some of its bugs are fixed and we'll maintain it during the course implementation.

### Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control 

### Install effort-controllers to use torque-control interface

    $ sudo apt-get install ros-noetic-effort-controllers 

### Add repos and install catkin_tools if not installed yet
```
$ sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

$ sudo apt-get update
$ sudo apt-get install python3-catkin-tools
```

### Download, create a workspace and build 

    $ mkdir -p elfin_ws/src
    $ cd ~/elfin_ws/src
    $ git clone https://github.com/tau-alma/edu-elfin_simulation.git
    $ cd ~/elfin_ws/
    $ catkin build
    $ source devel/setup.bash


### Run examples
Motion controllers in joint space

    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=gravity_comp_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=time_delay_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_clik_controller

If you want to use motion and force controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_gazebo elfin3_experiment1_world.launch controller:=adaptive_impedance_controller
    or
    $ roslaunch elfin_gazebo elfin3_experiment2_world.launch controller:=adaptive_impedance_controller

If you want to plot data in rqt graph, use rqt_plot.launch file. Customize perspective files to plot data you need.

    $ roslaunch rqt_plot.launch controller:=gravity_comp_controller



## Notes
- If you are using VMWare and gazebo keeps crashing during run time, try the following:

```
$ echo "export SVGA_VGPU10=0" >> ~/.bashrc
``` 
- before running any of the sample codes, you also need to issue the following command from your /catkin_ws directory so the built nodes and launch files could be found in terminal:


```
$ source devel/setup.bash
```

- If your gazebo environment and the robot is too dark, find the corresponding world file (for example empty.world) location and add the followings to the world:
```
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.25 0.25 0.25 1</background>
      <shadows>false</shadows>
    </scene>
```

## Docker

There is a docker configuration file that can be used to run the simulation environment and scripts inside of the docker containers


To build the docker container use the build script
```
./build_docker.sh
```

and run the images from the root of the repository, this will copy the current version of your scripts to the docker image.
```
docker run -it --env="DISPLAY" --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --volume="/tmp/.X11-unix:/tmp/.X11-unix" -v .:/root/src --privileged --net=host edu-elfin:latest
```
Inside of the docker image to access the environment use following:

```
cd root
catkin build
roslaunch elfin_gazebo elfin3_empty_world.launch controller:=gravity_comp_controller
```


## References
1. [ros_Control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin Robot](http://wiki.ros.org/Robots/Elfin)
4. [Elfin Simulation package](https://github.com/modulabs/arm-control)
5. [ROS](http://wiki.ros.org/)


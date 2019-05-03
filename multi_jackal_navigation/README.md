This document describes the steps necessary to set up a multi-master network for two Jackals to receive goal commands from a ground station computer and navigate to those goals independently in a mapped environment.

The generalized navigation control-flow between computers is displayed in the figure below. 
  
<p align="center">
    <img src="https://github.com/RCConnolly/Jackal_Velodyne_Duke/wiki/images/comm_control_flow.png" width="400">
 </p>

# Set-up

## Workstation Configuration

This workstation for project was set-up using Ubuntu 14.04 with ROS Indigo. Be sure that the following steps have been completed on your workstation before proceeding:

1. Install & Configure ROS - after installation of ROS Indigo, create a catkwin worspace to develop in by following these [instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

2. Install Jackal specific ROS metapackages for simulating the robot with Gazebo and visualizing with Rviz 

    ```
    sudo apt-get install ros-indigo-jackal-simulator ros-indigo-jackal-desktop ros-indigo-jackal-navigation
    ```
  
3. Obtain a clone of this GitHub repository within the catkin workspace _src_ folder:
    ```
    cd PATH/TO/catkin_ws/src
    git clone https://github.com/RCConnolly/Jackal_Velodyne_Duke.git 
    ```
The above packages should allow for visualization and autonomous navigation of a single Jackal. In addition to these dependencies, multi-jackal navigation requires installation of the [multi-master ROS package](http://wiki.ros.org/multimaster_fkie). 

4. To do this, run the following command, and if using a ROS distribution other than Indigo, replace `indigo` with the name of that distribution.
    ```
    sudo apt-get install ros-indigo-multimaster-fkie
    ```

## Jackal Initial-Setup

If not already completed, follow the [initial set-up procedure](../../../#jackal-initial-setup) for each Jackal.

Next, complete each section listed on the [single jackal navigation](../../README.md) page. This ensures that the _pointcloud2laserscan_ package is installed and that each Jackal is properly configured for autonomous navigation.


## Modifying Jackal's Local Files

The multi-master framework requires modification of several system files located on each Jackal and on the workstation to ensure that each computer can wirelessly communicate with each other and that each computer knows which ROS network it belongs to.

### Modifying System Files

A detailed description of establishing a [multi-master](http://wiki.ros.org/multimaster_fkie) network is described [here](http://www.iri.upc.edu/files/scidoc/1607-Multi-master-ROS-systems.pdf), but these instructions are not fully applicable to this project. The reason being is that the multi-master ROS package is intended to use multi-cast broadcasting, however, the communication for this project is via WiFi, which does not use multi-cast by default.

For this project, the following modifications to local files are required:

 1. Edit the /etc/hosts file to include hostname-IP pairs of each computer in the multi-master network.
 2. Ensure that the computer's /etc/hostname matches the local hostname in the /etc/hosts file.
 3. Add to the ~/.bashrc file to set the [ROS_MASTER_URI](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_MASTER_URI) variable to match the hostname of the computer it belongs to. The ~/.bashrc file is sourced every time a new terminal window is opened, and can be directly sourced with the following command `source ~/.bashrc` so that opening a new window is not required.

For reference, the relevant lines of these system files after modification are listed below for the Jackal-2 robot and the workstation desktop (with in-line comments after the # symbol):

**Jackal's /etc/hosts**:
```
127.0.0.1	localhost

# Jackal's are set up with static IP so use that instead of 127.0.1.1 as recommended here:
# https://www.debian.org/doc/manuals/debian-reference/ch05.en.html#_the_hostname_resolution
10.194.128.24 	jackal-2.wireless.duke.edu jackal-2

# The following lines are for multi-master framework for Jackal robots
10.194.128.23 jackal-1
10.236.66.137 ramaDesktop
```

**Jackal's /etc/hostname**:
`jackal-2`

**Jackal's ~/.bashrc**:
`export ROS_MASTER_URI=http://10.194.128.24:11311`

**Desktop's /etc/hosts**:
```
127.0.0.1	localhost
127.0.1.1	dukerama-Precision-WorkStation-T5500

# The following lines are for multi-master framework for Jackal robots
10.194.128.23 jackal-1
10.194.128.24 jackal-2
10.236.66.137 ramaDesktop
```
**Desktop's /etc/hostname**:
`dukerama-Precision-WorkStation-T5500`

**Desktop's ~/.bashrc**:
```
source /opt/ros/indigo/setup.bash
source ~/jackal_catkin_ws/devel/setup.bash

# ROS_MASTER_URI is used to tell nodes where the master is
export ROS_MASTER_URI=http://10.236.66.137:11311 # desktop IP non-static, may change on computer start-up

# ROS_HOSTNAME set the network address of ROS nodes
export ROS_HOSTNAME=ramaDesktop
```

To obtain the IP address of a computer, use the command `ip addr`. The IP addresses of each hostname listed in the /etc/hosts file should match the current IP address of the computer that the hostname is referencing. For our network, each Jackal was assigned a static IP so this number should only need to be set once, however, the desktop's IP is not static, meaning that it's IP address will need to be re-checked after shutdown or restart.

### (OPTIONAL, ADVANCED) Modifying Jackal's default navigation parameters

Each Jackal robot comes pre-installed with [packages](https://github.com/jackal/jackal) that describe the ROS parameters of the robot, such as velocity limits and model descriptions.

While the default parameters should work for the robot, we've found a few modifications that may improve the navigation performance for our specific application. Since the packages are tracked through Github, it is recommended to create a new branch from the _master_ branch and make any changes in that new branch. This way default settings can be restored by checking out the _master_ branch again.

First, in order to modify the Jackal's parameters the repository needs to be cloned into that Jackal's catkin workspace source folder. Check to see if the alredy exists on the Jackal by ssh'ing into it and typing
```
cd ~/catkin_ws/src/
ls
```

If `jackal` is listed as one of the directories then that repository is already cloned, if it is not there clone it with the following commands after ssh'ing into the Jackal.

```
cd ~/catkin_ws/src
git clone https://github.com/jackal/jackal.git
```

Note that if the jackal package was already installed via `sudo apt-get`, then you may need to change how the package is sourced in the ~/.bashrc file.

The following modifications were made to improve the translational and rotational accuracy when achieving a goal using move_base. Note that if these tolerances are too small, the Jackal will have a difficult time achieving that precision and will consider the goal unreachable after attempting recovery behaviors.

In 'jackal/jackal_navigation/params/base_local_planner_params.yaml':

  ```
  # Goal Tolerance Parameters
  yaw_goal_tolerance: 0.08 # default: 0.157
  xy_goal_tolerance: 0.15 # default: 0.25
  ```

Note that after making changes to any of the jackal navigation files, you will likely need to restart the robot for them to take effect.

# Navigation

The general procedure to sendings goals from the desktop to one or multiple robots is:
1. Initialize the necessary ROS nodes by using launch files
2. Share the required ROS topics between computers
3. Finally running the navigation script.

The details of each of these steps are described in the sections below, specifically for running a Jackal in simulation and navigating a real Jackal robot.

## Single Jackal Simulation

A single robot can be simulated in a virtual environment and sent goals with the _multi_robot_nav.py_ script. Clearpath Robotics provides [instructions](http://www.clearpathrobotics.com/assets/guides/jackal/simulation.html) for simulating the Jackal in a Gazebo 3D environment. The Jackal can be simulated with Gazebo, visualized with RViz, and send navigation commands via the_multi_jackal_nav.py_ script with the following commands:

```
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
roslaunch jackal_navigation amcl_demo.launch
cd jackal_catkin_ws/src/Jackal_Velodyne_Duke/navigation
./jackal_goal_server Jackal1
roslaunch jackal_viz view_robot.launch config:=localization
./multi_jackal_nav Jackal1
```

## Single Jackal Real-World

Navigating a robot within a saved map can be accomplished by following the listed steps below. Note that each command needs to be run in an individual terminal window, and the commands listed as "on Jackal" should be run by first ssh'ing into that Jackal. The commands below specify the Jackal-1 robot, but they can be used for the Jackal-2 by replacing `Jackal1` with `Jackal2`.

  1. On workstation: Tell the desktop to discover the necessary topics of the Jackal.
  ```
  roslaunch jackal_velodyne_duke jackal_discovery.launch
  ```
  2. On Jackal-1: Convert pointcloud messages to 2D laserscans, launch the amcl localiztion node, launch the Jackal's goal server, and discover the desktop's master node. Then sync the required topics with the desktop.
  ```
  roslaunch jackal_velodyne_duke velodyne_amcl.launch robot_name:=Jackal1 x:=0 y:=0 yaw_rad:=-1.7 discovery:=true
  roslaunch jackal_velodyne_duke desktop_sync.launch jackal_name:=Jackal1
  ```
  
  3. On Workstation: Sync the topics from Jackal and run the navigation script. 
  ```
  roslaunch jackal_velodyne_duke jackal_sync.launch 
  cd jackal_catkin_ws/src/Jackal_Velodyne_Duke/navigation/
  ./multi_jackal_nav.py Jackal1
  ```
  
  ## Multiple Jackals Real-World
  
Navigating two robots simulatenously within a saved map can be accomplished with the following steps. Note that each command needs to be run in an individual terminal window, and the commands listed as "On Jackal" should be run by first ssh'ing into that Jackal.

Note that the x, y, yaw_rad specified in the velodyne_amcl.launch file specify the pose to intialize within the _hudson_back_test.yaml_ map.

On Workstation: Tell the desktop to discover the necessary topics for navigating two jackals
```
roslaunch jackal_velodyne_duke jackal_discovery.launch dual_discover:=true
```
On Jackal1: Convert pointcloud messages to 2D laserscans, launch the amcl localiztion node, launch the Jackal's goal server, and discover the desktop's master node. Then sync topics with the desktop.
```
roslaunch jackal_velodyne_duke velodyne_amcl.launch robot_name:=Jackal1 x:=0 y:=0 yaw_rad:=-1.7 discovery:=true
roslaunch jackal_velodyne_duke desktop_sync.launch jackal_name:=Jackal1
```
On Jackal2: Same as the previouse two commands, but for Jackal2 with a different initialization position.
```
roslaunch jackal_velodyne_duke velodyne_amcl.launch robot_name:=Jackal2 x:=-1 y:=0.23 yaw_rad:=-1.7 discovery:=true
roslaunch jackal_velodyne_duke desktop_sync.launch jackal_name:=Jackal2
```
On Workstation: Sync the topics from both Jackals. Then run the navigation script that coordinates navigation commands to the Jackals. Note that each goal location is specified within this file, so ensure that the goals match the map you are localized into
```
roslaunch jackal_velodyne_duke jackal_sync.launch dual_sync:=true
./multi_jackal_nav.py Jackal1 Jackal2
```

# Troubleshooting

There are a few general checks that you should make when encountering an error. Often times, the errors that arise with these issues are not indicative of the actual problem.

1. If any changes have been made to any of the packages within the catkin worskpace that require updating dependencies or rebuilding executables (not including changes to a python script), then be sure to re-make the catkin workspace.

```
cd ~/catkin_ws # or ~/jackal_catkin_ws for the workstation
catkin_make
```

2. The two Jackals have their IP's set to static on Duke's network, however, the workstation IP is non-static. Therefore, whenever the computer restarts or turn on after a shutdown, its IP may have changes. Ensure that the IP address of each computer is correct in each of the computer's /etc/hosts file.

3. Since several ROS nodes are created on start-up of the robot, any changes to these nodes will require a restart to the Jackal.

4. Several of the packages, including this jackal_velodyne_duke package, are cloned GitHub repositories. Therefore, there may be several development branches that differ between another. Always ensure that you are on the correct Github branch, which can be checked with the command `git branch` and that the branch is up-to-date with `git pull`.

5. Occasionally when running RVIz or the Gazebo simulation, the top of the Jackal will not be visualized and a corresponding error is printed to the terminal saying that _jackal-fenders.stl_ cannot be located. This is because on older versions of the jackal description package, this file is named _jackal_fender.stl_. If this error occurs, simply switch the name of the _jackal-fender.stl_ file to _jackal-fenders.stl_.

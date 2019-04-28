This document describes the steps necessary to set up a multi-master network for two Jackals to receive goal commands from a ground station computer and navigate to those goals independently in a mapped environment.

The generalized navigation control-flow between computers is displayed in the figure below. 
  
<p align="center">
    <img src="https://github.com/RCConnolly/Jackal_Velodyne_Duke/wiki/images/comm_control_flow.png" width="400">
 </p>

# Set-up

The first step is to follow the [initial set-up procedure](../../../#jackal-initial-setup).

Next is configuring the multi-master network for navigation, which requires several steps, including installing file dependencies and modifying existing files on the Jackal robot.  Instructions for these steps are detailed below.

## Installing dependencies

Certain additional ROS packages are required for navigation and simulation of the Jackal robots. First, run the following commands if these packages are not already installed. [TODO jackal set-up and navigation dependencies].

The above packages should allow for visualization and autonomous navigation of a single Jackal. In addition to these dependencies, multi-jackal navigation requires installation of the [multi-master ROS package](http://wiki.ros.org/multimaster_fkie). To do this, run the following command `sudo apt-get install ros-indigo-multimaster-fkie`. If using a ROS distribution other than Indigo, replace `indigo` with the name of that distribution.

## Modifying Jackal's Local Files

### Modifying System Files

A detailed description of establishing a [multi-master](http://wiki.ros.org/multimaster_fkie) network is described [here](http://www.iri.upc.edu/files/scidoc/1607-Multi-master-ROS-systems.pdf), but these instructions are not fully applicable to this project. The reason being is that the multi-master ROS package is intended to use multi-cast broadcasting, however, the communication for this project is via WiFi, which does not use multi-cast by default.

For this project, the following modifications to local files are required:

 1. Edit the /etc/hosts file to include hostname-IP pairs of each computer in the multi-master network.
 2. Ensure that the computer's /etc/hostname matches the local hostname in the /etc/hosts file.
 3. Add to the ~/.bashrc file to set the [ROS_MASTER_URI](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_MASTER_URI) variable to match the hostname of the computer it belongs to. The ~/.bashrc file is sourced every time a new terminal window is opened, and can be directly sourced with the following command `source ~/.bashrc` so that opening a new window is not required.

For reference, these are the system files after modification for one of the Jackals (with in-line comments after the # symbol):

**Jackal's /etc/hosts**:
```
127.0.0.1	localhost

# Jackal's are set up with static IP so use that instead of 127.0.1.1 as recommended here:
# https://www.debian.org/doc/manuals/debian-reference/ch05.en.html#_the_hostname_resolution
10.194.128.24 	jackal-2.wireless.duke.edu jackal-2

# The following lines are for multi-master framework for Jackal robots
10.194.128.23 jackal-1
10.236.66.137 ramaDesktop

# The following lines are desirable for IPv6 capable hosts
::1     localhost ip6-localhost ip6-loopback
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
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

### Modifying default packages installed on Jackal

Each Jackal robot comes pre-installed with [packages](https://github.com/jackal/jackal) that describe the ROS parameters of the robot, such as velocity limits and model descriptions.

While the default parameters should work for the robot, we've found a few modifications that improve the navigation performance for our specific application. Since the packages are tracked through Github, it is recommended to create a new branch from the _master_ branch and make any changes in that new branch. This way default settings can be restored by checking out the _master_ branch again.

The following modifications were made to the Jackal packages for each robot:

`TODO INCLUDE ANY JACKAL FILE MODIFICATIONS SUCH AS UPDATE FREQUENCIES AND VELOCITY LIMITS`

Note that after making changes to any of this files, you will likely need to restart the robot for them to take effect.

# Navigation

There general procedure to sendings goals from the desktop to one or multiple robots is:
1.  Initialize the necessary ROS nodes by using launch files
2. Share the required ROS topics between computers
3. Finally running the navigation script.

The details of each of these steps are described in the sections below, specifically for running a Jackal in simulation and navigating a real Jackal robot.

## Single Jackal Simulation

A single robot can be simulated in a virtual environment and sent goals with the _multi_robot_nav.py_ script. Clearpath Robotics provides [instructions](http://www.clearpathrobotics.com/assets/guides/jackal/simulation.html) for simulating the Jackal in a Gazebo 3D environment. Once the necessary simulation packages are installed, follow the [navigation instructions](http://www.clearpathrobotics.com/assets/guides/jackal/navigation.html) published by Clearpath Robotics to set-up the simulated environment. The _multi_jackal_nav.py_ script can then be used to send a sequence of goal commands to the simulated Jackal.

## Real-World

Navigating both robots within a saved map can be accomplished with the following steps:

`TODO INCLUDE NAVIGATION STEPS`


# Troubleshooting

check that dekstop's IP matches on each Jackal
catkin_make
restart Jackal

Jackal_Velodyne_Duke
========

Experiences with navigating multiple Jackal robots (equipped with Velodyne LiDAR) at Duke University.

-----
Description
-----
This repository contains instructions and other documentation developed at Duke University, for navigating multiple Jackal robots by [Clearpath](https://www.clearpathrobotics.com) equipped with standard hardware plus the [Velodyne LiDAR](https://velodynelidar.com/). 

  <p align="center">  
  <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/figures/odom_navg.png" width="800"/>
  </p>


-----
Jackal Initial Setup
-----

A detailed description of setting up the Jackal is provided by Clearpath [here](https://www.clearpathrobotics.com/assets/guides/jackal/network.html). The steps we used are summarized below.

  First, connect Jacakal by wire to your LAN. `ssh` into the machine and connected your local Wifi via `wicd-curses`. Make sure _the automatic connection_ is checked in the configuration of Wifi signal. We also chose to set `static_IP` for each Jackal and enabled DHCP. 

  Unplug the wire and reboot Jackal to make sure that Jackal is connected to your preferred Wifi. We verify this by `ping`  and `ssh` into Jackal.

  After you `ssh` into Jackal, try `rostopic list`. You will see that there are already _many_ topics being published. The reason is that there is a startup script that runs once Jackal is powered on, see [here](https://www.clearpathrobotics.com/assets/guides/jackal/startup.html) from Clearpath for details. In other words, a ROS master is up and running at Jackal. 

-----
Navigation
-----

There are two types of navigation procedures that can be performed. The first is navigating a single Jackal robot and using RViz on a desktop to visualize it's sensed environment and send commands to the robot. The second is performing simultaneous navigation of two Jackal robot's using the desktop as a ground station computer that interacts with each robot. Details regarding each of these navigation procedures are located in their repsective folders, namely [navigation](navigation) and [multi_jackal_navigation](multi_jackal_navigation).

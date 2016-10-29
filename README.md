Jackal_Velodyne_Duke
========

Experiences with Jackal robot (equipped with Velodyne LiDAR) at Duke University.

-----
Description
-----
This repository contains the memos, logs and packages developed at Duke University, for Jackal robots by [Clearpath](https://www.clearpathrobotics.com) equipped with standard hardware plus the [Velodyne LiDAR](velodynelidar.com/). 

<p align="center">  
  <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/figures/jackal_lidar.jpg" width="700"/>
</p>


-----
Jackal Initial Setup
-----
This markdown contains the initial steps we took to set up the Jackal, which involves mainly three steps. Afterwards, you should be able to drive Jackal manually and visualize the pointcloud from Velodyne via Rviz, see [example video](https://vimeo.com/185578563).

* Step 1. Read [this](https://www.clearpathrobotics.com/assets/guides/jackal/network.html) from Clearpath first.

Connect Jacakal by wire to your LAN. `ssh` into the machine and connected your local Wifi via `wcid-curses`. Make sure _the automatic connection_ is checked in the configuration of Wifi signal. We also choose to set `static_IP` which works fine for a single Jackal (but not so well when multiple Jackal appears). We also enabled DHCP. 

Unplug the wire and reboot Jackal to make sure that Jackal is connected to your preferred Wifi. We verify this by `ping`  and `ssh` into Jackal.

After you are confident that your Jakcal connects automatically to the Wifi with a fixed or known IP (which you can monitor from your Router administer page), move to the next step. 


* Step 2. After you `ssh` into Jackal, try `rostopic list`. You will see that there are already _many_ topics being published. The reason is that there is a startup script that runs once Jackal is powered on, see [here](https://www.clearpathrobotics.com/assets/guides/jackal/startup.html) from Clearpath for details. In other words, a ROS master is up and running at Jackal. 

Now we want to make these messages available to other ROS nodes running within the same LAN. We have a workstation with `Ubuntu 14.04` and `ROS Indigo`. We set up [ROS across multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines). Since a ROS master starts automatically at Jackal, we decided to put ROS Master at Jackal (which however will be a problem if multiple Jackals are present. Instead, we plan to use a [multiple-master scheme]() ). Basically, we add to Jackal's and workstation's  `~/.bashrc` the following:

```
# at Jackal, ~/.bashrc
export ROS_IP = x.x.x.x1

# at workstation, ~/.bashrc
export ROS_IP = x.x.x.x2
export ROS_MASTER_URI = x.x.x.x1

```

After the workstation is subscribed to the ROS master on Jackal, you should be able to see all the ROS messages on your workstation.

* Step 3. Visualize Jackal in Rviz.

Before launching Rviz, you need to do these things:

# **copy** the customized packages from `jackal_name:~/catkin_ws` to `workstation:~/catkin_ws`, via `scp`.

# add in workstation's `/etc/hosts` the computer name and IP of Jackal (_critical_):

```
# at workstation, ~/etc/hosts
x.x.x.x1	  Jackal_computer_name
```

# install `jackal_viz` from [ros wiki](http://wiki.ros.org/jackal_viz). Then launch the visualization:

```
roslaunch jackal_viz view_robot.launch
```

You will see the Jackal 3D model as below and you can add sensor data by topic (as shown in the last part of the [offcial tutorial](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html)). For velodyne, it is `/pointcloud2` . Drive the Jackal around by `interact`.






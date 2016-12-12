Jackal_Velodyne_Duke [navigation]
========

Experiences with Jackal robot (equipped with Velodyne LiDAR) at Duke University.

-----
Description
-----
This repository contains the memos, logs and packages developed at Duke University, for Jackal robots by [Clearpath](https://www.clearpathrobotics.com) equipped with standard hardware plus the [Velodyne LiDAR](velodynelidar.com/). 

After following the basic [Jackal setup tutorial](https://github.com/MengGuo/Jackal_Velodyne_Duke), you should be able to see Jackal via Rviz and manually navigate it. 
In this part, we focused on testing the official [Jackal navigation package](https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation), which is built on the [ROS navigation stack](http://wiki.ros.org/navigation), particularly the [`move_base`](http://wiki.ros.org/move_base?distro=kinetic) package. The test consists of _three_ parts: 1. autonomous navigation via Ros `move_base`; 2. mapping via `gmapping`; and 3. self-localization via `amcl`.

Make sure you have read the [official navigation tutorial](https://www.clearpathrobotics.com/assets/guides/jackal/navigation.html) from clearpath.



-----
Part one: autonomous navigation
-----

Clearpath provides a [Jackal navigation package](https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation), which is great. So install it either via `apt-get install` or _download the source_ from github and `catkin make`. The first method ensures it is always up-to-date via package management while the second allows local modification.   This part corresponds to the **NAVIGATION WITHOUT A MAP** part of [official Jackal navigation](https://www.clearpathrobotics.com/assets/guides/jackal/navigation.html).

  * **Try default navigation package.**

  Jackal navigation should be able to run right out of the box, **if** a 2D laser is equipped for Jackal, such as the SICK scanner. Namely, `ssh` into Jackal and run the following node:

  ```
  roslaunch jackal_navigation odom_navigation_demo.launch
  ```

  which basically activate the move_base node with the appropriate configuration parameters for the Jackal dynamic model, sensor model, local and global planner. More details can be found [here](https://github.com/jackal/jackal/blob/indigo-devel/jackal_navigation/launch/include/move_base.launch).

  Then in your workstation, run Rviz with the configuration `navigation`:

  ```
  roslaunch jackal_viz view_robot.launch config:=navigation
  ```

  Try to click on the 2D navigation and set the goal point behind an obstacle, you will see that Jackal follows a straight path and **collide** into the obstacle. Since `move_base` is designed to handle both static and dynamic obstacles, we realize there is something wrong with the interface between `Jackal navigation` and `move_base`.

  * **Add pointcloud_to_laserscan.**
  
  So if you take a look at the [configuration of move_base](https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation/params), particularly for [costmap_common_params](https://github.com/jackal/jackal/blob/indigo-devel/jackal_navigation/params/costmap_common_params.yaml), notice the following line

  ```
  obstacles_layer:
  observation_sources: scan
  scan: {sensor_frame: front_laser, data_type: LaserScan, topic: front/scan, marking: true, clearing: true, min_obstacle_height: -2.0, max_obstacle_height: 2.0, obstacle_range: 2.5, raytrace_range: 3.0}
  ```
  
  which says the obstacle layer relies on laser scan from topic `/front/scan`. It is easy to check that no nodes are publishing to it.

  Thus we use a ROS package [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan) that transform the point loud from Velodyne to virtual 2D laser scan. Partially, after installing `pointcloud_to_laserscan`, we create the following launch file `point2laser.launch` within its installation directory:

  ```
  <launch>
  <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">

    <remap from="cloud_in" to="/velodyne_points"/>
    <remap from="scan" to="/front/scan"/>
    <rosparam>
        transform_tolerance: 0.01
        min_height: 0.25
        max_height: 0.75

        angle_min: -3.1415
        angle_max: 3.1415
        angle_increment: 0.01
        scan_time: 0.1
        range_min: 0.9
        range_max: 130
        use_inf: true
        concurrency_level: 0
    </rosparam>

  </node>
  </launch>
  ```


  After launching it in another terminal of Jackal computer, you can see grey 2D laser points in Rviz by subscribing to `/front/scan`, as shown below:

<p align="center">  
  <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/figures/pointcolud2laserscan.png" width="500"/>
</p>



  It means we have successfully convert the pointcloud from `/Velodyne_points` to 2D laserscan `/front/scan`, which is the **only** allowed sensory input for gmapping and amcl used later.

  * **Add tf transformation.**

  However even with the `pointcloud_to_laserscan` enabled, repeat the point-to-point navigation in Rviz again. You will see that that the Jackal still runs directly into obstacles. Then we realize there is a warning message appearing in the `Jackal_navigation` terminal:
  ```
  the message from [/odom /front_laser] 100% dropped.
  ```


  It means that the `tf` relation from `/velodyne` to `/front_laser` is not constructed. If we print out the transformation relation from `/odom` to `/velodyne`, the results are normal (but no transformation relation from `/odom` to `/front_laser`), which is crucial for `move_base`. Thus we add a **static identity mapping from `/velodyne` to `/front_laser`** in the same `point2laser.launch` file:

  ```
  <node pkg="tf" type="static_transform_publisher" name="velodyne_to_front_laser" 
args="0 0 0 0 0 0 /velodyne /font_laser 100" />
  ```
  
  Thus the complete `point2laser.launch` file is [here](https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/point2laser.launch). Once it is done, re-launch it. You can verify that the warning message is gone and the `tf` relation from `/odom` to `/front_laser` is established. Then launch the `odom_navigation.launch` again:

  ```
  roslaunch jackal_navigation odom_navigation_demo.launch
  ```
  You will see that now automatons navigation is working with collision avoidance, see [[video1]](https://vimeo.com/189086502), [[video2]](https://vimeo.com/189087199).

  <p align="center">  
  <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/figures/odom_navg.png" width="800"/>
  </p>
  
-----
Part two: mapping
-----

  Once the `odom_navigation.launch` is working with static and dynamic collision avoidance. You may move the second part of the navigation task, i.e., to make a map using `gmapping`. This part corresponds to the **MAKING A MAP** part of [official Jackal navigation](https://www.clearpathrobotics.com/assets/guides/jackal/navigation.html).

  First you need to launch the modified `point2laser.launch` file from [here](https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/point2laser.launch). Then you can launch the `gmapping_demo.launch` from the [Jackal navigation package](https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation) at the Jackal onboard computer:

  ```
  roslaunch jackal_navigation gmapping_demo.launch
  ```

  Then you can visualize everything on the desktop via:
  
  ```
  roslaunch jackal_viz view_robot.launch config:=gmapping
  ```
  Drive Jackal across the workspace you want to map and save the map in the end using
  ```
  rosrun map_server map_saver -f workspace
  ```
  This will create a `workspace.yaml` and `workspace.pgm` file in your current directory. Note that any static it came across on the way will belong to the global static map.
  
  <p align="center">  
  <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/figures/gmapping.png" width="300"/>
  </p>
  
-----
Part three: localization
-----


  Once you have created a static global map `workspace.yaml`. You may move the third part of the navigation task, i.e., to self-localization using `amcl`. This part corresponds to the **NAVIGATION WITH A MAP** part of [official Jackal navigation](https://www.clearpathrobotics.com/assets/guides/jackal/navigation.html).

  First you need to launch the modified `point2laser.launch` file from [here](https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/point2laser.launch). Then you can launch the `amcl_demo.launch` from the [Jackal navigation package](https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation) at the Jackal onboard computer:

  ```
  roslaunch jackal_navigation amcl_demo.launch map_file:=/path/to/my/workspace.yaml
  ```

  Then you can visualize everything on the desktop via:
  
  ```
  roslaunch jackal_viz view_robot.launch config:=localization
  ```
  You can now keep track of the real-time localization of the robot via the topic `/amcl_pose`. We use the Python [script](https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/amcl_pose_to_2D_Euler.py) to translate the `PoseWithCovarianceStamped` message from `/amcl_pose` to `(robot_pose_x,robot_pose_y,robot_orientation)`.
  Moreover, we use another Python [script](https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/seq_goal_nav.py) to **set initial pose estimation** and *send a sequence of global waypoints*, without using the Rviz. 


  <p align="center">  
  <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/navigation/figures/amcl.png" width="300"/>
  </p>
Jackal_Velodyne_Duke [navigation]
========

Experiences with Jackal robot (equipped with Velodyne LiDAR) at Duke University.

-----
Description
-----
This repository contains the memos, logs and packages developed at Duke University, for Jackal robots by [Clearpath](https://www.clearpathrobotics.com) equipped with standard hardware plus the [Velodyne LiDAR](velodynelidar.com/). 

We focused on testing the offical [Jackal navigation package](https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation), which  is built on the [ROS navigation stack](http://wiki.ros.org/navigation), particularly the [`move_base`](http://wiki.ros.org/move_base?distro=kinetic) package.

After following the basic Jackal setup tutorial you should be able to see Jackal via Rviz and manually navigate it. The test consists of _three_ parts: 1. autonomous navigation via Ros `move_base`; 2. mapping via `gmapping`; and 3. self-localization via `amcl`.

Make sure you have read the official tutorial from clearpath.



-----
Part one: autonomous navigation
-----

Clearpath provides a jackal navigation package, which is great. So install it either via apt-get install or download the source from github and catkin make. The first method ensures it is always up-to-date while the second allows local modification.

Jackal navigation should be able to run right out of the box, if a 2D laser is equipped for Jackal, such as the SICK scanner. Namely, ssh into Jackal and run the following node:

which basically activate the move_base node with the appropriate configuration parameters for the jackal dynamic model, sensor model, local and global planner. More details can be found here.

Then in your workstation, run Rviz with the configuration  navigation:

Try to click on the 2D navigation and set the goal point behind an obstacle, you will see that Jackal follows a straight path and collide into the obstacle. Since move_base is designed to handle both static and dynamic obstacles, there is something wrong with the interface.

So if you take a look at the configuration of move_base, the following line

says it is listening to /front/scan as sensory input, while no nodes are publishing to it. Thus we use a ROS package pointcloud2laserscan that transform the point loud from Velodyne to virtual 2D laser scan. Partially, after installing ...., we create the following launch file within its installation directory:


After launching it in another terminal of Jackal computer, you can see grey 2D laser points as shown below:

It means we have successfully convert the point loud to 2d laser, which is the only sensory input for gmapping and amcl used later.




-----
Part two: mapping
-----




-----
Part three: localization
-----

Jackal_Velodyne_Duke [Xbee communication]
========

Experiences with Jackal robot (equipped with Velodyne LiDAR) at Duke University.

-----
Description
-----
This repository contains the memos, logs and packages developed at Duke University, for Jackal robots by [Clearpath](https://www.clearpathrobotics.com) equipped with standard hardware plus the [Velodyne LiDAR](velodynelidar.com/).


We intend to transmit data from the Jackal robot to a stationary data center, via a mesh network of wireless nodes. The goal is to study the communication-aware motion planning problem of mobile robots.
Even though Jackal is equipped with powerful wifi/bluetooth communication modules, we use them only for the purpose of monitoring the operation status of Jackal.


In particular, we use several [Xbee radio](https://www.digi.com/) modules from Digi, particularly, series 2 with protocol Zigbee. The serial to micro-USB dongle from [SparkFun](https://www.sparkfun.com/products/11812)  is used.


  <p align="center">  
    <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/xbee_communication/figures/xbee.png" width="500"/>
  </p>


-----
Data transfer in AT mode via XCTU
-----

[XCTU](https://www.digi.com/products/xbee-rf-solutions/xctu-software/xctu) is the official GUI to configure Xbee easily. The [tutorial](http://www.digi.com/resources/documentation/digidocs/90001458-13/default.htm) from Digi is quite helpful.
We first tried sending data in AT mode from an end node to a coordinator node, via several relay nodes. 


-----
Data transfer in API mode via Xbee-Python
-----

In order to send data automatically without using the GUI, we use the [Xbee-Python](https://python-xbee.readthedocs.io/en/latest/) library.
Particularly, we send a `1000*1000` matrix from the end node via [`tx_end_node.py`](https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/xbee_communication/tx_end_node.py) and receive it at the coordinator via [`rx_coordinator.py`](https://github.com/MengGuo/Jackal_Velodyne_Duke/tree/master/xbee_communication).

  <p align="center">  
    <img src="https://github.com/MengGuo/Jackal_Velodyne_Duke/blob/master/xbee_communication/figures/rviz_xbee_map.png" width="500"/>
  </p>

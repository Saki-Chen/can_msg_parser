# can msg parser
## Introduction
This ros package contains some example for using converting raw CAN message using ros_esdcan_bridge.

## Dependency   
* ros esdcan bridge   
  https://github.com/Saki-Chen/ros_esdcan_bridge  

* apa_msgs   
  https://github.com/Saki-Chen/apa_msgs

* kvaser interface(optional)   
  https://github.com/astuff/kvaser_interface

## Build
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Saki-Chen/can_msg_parser
cd ../..
catkin_make
```

## Usage
```bash
source catkin_ws/devel/setup.bash
# if you are using bag for simulation without real can device, use simulation.
# if you are using real can device choose the device type.
roslaunch can_msg_parser can_driver.launch can_dev_type:=<esdcan or kvaser> simulation:=<true or false>
```
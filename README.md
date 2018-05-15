# ariac_project
ARIAC: Agile Robots for Industrial Automation Competition. The project is for implementing solutions to real-life problems in a manufacturing environment. The environment is simulated in gazebo using the osrf_gear package.

## Dependencies
1.```osrf_gear``` is the package which is the primary dependency.This can be downloaded from [here](http://wiki.ros.org/ariac/Tutorials/SystemSetup). 

2. Gazebo 7

3. ROS Indigo or ROS kinetic

4. Moveit! and Moveit! for ARIAC packages. Follow instructions from [here](http://wiki.ros.org/ariac/Tutorials/MoveItInterface). Choose the correct ROS version. After you are done, put ```source install/setup.bash``` from where you installed moveit packages in the ```~/.bashrc```. Make sure you check where Moveit is working properly and you can do planning by using following commands and following the tutorial page-
```
 rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/sample.yaml
 ```
 ```
 roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
 ```
 ```
 roslaunch ur10_moveit_config moveit_rviz.launch config:=true
 ```


## Installation
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/rishabh1b/ariac_project
cd ~/catkin_ws
catkin_make
```

## Running a Basic Demo
Following commands will run qualifier 1a

Terminal 1 - 
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch ariac_comp gear_bringup.launch
```
Terminal 2 - 
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch ariac_comp competition.launch
```
## Outputs 
| Picking From Bin  | Picking From Conveyor Belt|
| ------------------- | ----------------- |
| <img src="https://github.com/rishabh1b/ariac_project/blob/master/gifs/BlindPartPicking.gif">| <img src="https://github.com/rishabh1b/ariac_project/blob/master/gifs/ConveyorPicking.gif" width="400"/>|

| Part Placement  | Part Placement correction|
| ------------------- | ----------------- |
| <img src="https://github.com/rishabh1b/ariac_project/blob/master/gifs/OrientationFixing.gif">| <img src="https://github.com/rishabh1b/ariac_project/blob/master/gifs/OrientationFixingRepick.gif" width="400"/>|

The code base is capable of picking and placing the parts in correct orientation using only one logical camera.

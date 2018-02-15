# ariac_project
ARIAC: Agile Robots for Industrial Automation Competition. The project is for implementing solutions to real-life problems in a manufacturing environment. The environment is simulated in gazebo using the osrf_gear package.

## Dependencies
1.```osrf_gear``` is the package which is the primary dependency.This can be downloaded from [here](http://wiki.ros.org/ariac/Tutorials/SystemSetup). 
2. Gazebo 7
3. ROS Indigo or ROS kinetic

## Installation
```
mkdir -p ~/catkin_ws/src
git clone https://github.com/rishabh1b/ariac_project
cd ~/catkin_ws
catkin_make
```
## For Contributing
_Make sure you work in your own branch_

Follow these steps for first time -
```
git branch <your_branch_name>
git push -u origin <your_branch_name>
git checkout <your_branch_name>
```
Follow these steps for making changes and pushing into your branch - 
navigate to your ```catkin_ws```
```
git checkout <your_branch_name>
```
Make whatever changes you want in your folder and push it to this repo. Make sure ```catkin_make``` runs without any errors before pushing your changes-

```
git add .
git commit -m "A logical commit message"
git push origin <your_branch_name>

```

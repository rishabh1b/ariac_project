#include "pick_and_place/PickAndPlace.h"

PickAndPlace::PickAndPlace(ros::NodeHandle nh_)
	:_manipulatorgroup("manipulator") {
	this->nh_ = nh_;

	try {
    	this->tf_tray_to_world.waitForTransform("/world", "/agv1_load_point_frame", ros::Time(0), ros::Duration(20.0) );
  } catch (tf::TransformException &ex) {
    	ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
    	ros::Duration(1.0).sleep();
  }

  try {
    this->tf_tray_to_world.lookupTransform("/world", "/agv1_load_point_frame", ros::Time(0), (this->_tray_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

  _tray_location_x = _tray_to_world_.getOrigin().x();
  _tray_location_y = _tray_to_world_.getOrigin().y();
  _tray_location_z = _tray_to_world_.getOrigin().z();


  _manipulatorgroup.getCurrentState()->copyJointGroupPositions(_manipulatorgroup.getCurrentState()->getRobotModel()->getJointModelGroup(_manipulatorgroup.getName()), home_joint_values);


  home_joint_values[4] = M_PI;
  // home_joint_values[0] = 2;

  // ROS_INFO_STREAM("Actuator Position : " << home_joint_values[1]);
  ROS_INFO_STREAM("Value of Each joint: " << home_joint_values[1] << " , " << home_joint_values[2] << " , " << home_joint_values[3] << " , " << home_joint_values[4] << " , " <<home_joint_values[5] << " , " << home_joint_values[6] );
  // initialSetup();

}
void PickAndPlace::initialSetup() {

  ROS_INFO("Getting into the Home Position");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  sleep(2.0);

  _manipulatorgroup.setJointValueTarget(home_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(10.0);
}



int main(int argc, char* argv[]) {
	ros::init(argc, argv, "simple_pick_and_place");
	ros::NodeHandle n;

	PickAndPlace pickPlace(n);

	return 0;
}
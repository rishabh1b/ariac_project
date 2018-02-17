#include "pick_and_place/PickAndPlace.h"

PickAndPlace::PickAndPlace(ros::NodeHandle nh_, double* initialjoints, double z_offset_from_part)
	:_manipulatorgroup("manipulator") {
	this->nh_ = nh_;
	this->_z_offset_from_part = z_offset_from_part;

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

  ROS_INFO_STREAM("home_joint_values[1]: " << home_joint_values[1]<< "\n");

  for (size_t i = 1; i < 7; i++) {
  	// ROS_INFO_STREAM("Joint Number: " << i << "\n");
  	home_joint_values[i] = initialjoints[i];
  }

  // ROS_INFO_STREAM("Actuator Position : " << home_joint_values[1]);
  ROS_INFO_STREAM("Value of Each joint: " << home_joint_values[1] << " , " << home_joint_values[2] << " , " << home_joint_values[3] << " , " << home_joint_values[4] << " , " <<home_joint_values[5] << " , " << home_joint_values[6] );
  initialSetup();

}
void PickAndPlace::initialSetup() {

  ROS_INFO("Getting into the Home Position");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

  _manipulatorgroup.setJointValueTarget(home_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(3.0);
  tf::StampedTransform transform;
  tf::TransformListener listener;
  /*geometry_msgs::PoseStamped currPose = _manipulatorgroup.getCurrentPose();
  geometry_msgs::Point currPoint = currPose.pose.position;
  _home_orientation = currPose.pose.orientation;*/

  // ROS_INFO_STREAM("The current Point x :" << currPoint.x << ", " << currPoint.y << ", " << currPoint.z);

  listener.waitForTransform("linear_arm_actuator","ee_link",ros::Time(0), ros::Duration(2));
      try{
        listener.lookupTransform("/linear_arm_actuator", "ee_link",  
	                  ros::Time(0), transform);
       }
      catch (tf::TransformException ex){
	   ROS_ERROR("%s",ex.what());
	   ros::Duration(1.0).sleep();
  }
  _home_orientation.x = transform.getRotation().x();
  _home_orientation.y = transform.getRotation().y();
  _home_orientation.z = transform.getRotation().z();
  _home_orientation.w = transform.getRotation().w();

}

void PickAndPlace::pickNextPart() {
	ROS_INFO("Picking the First Part");
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;
	// Starting Postion before picking
  	target_pose1.position.x = -0.233;
  	target_pose1.position.y = -0.6;
  	target_pose1.position.z = 0.723 + _z_offset_from_part;
	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(3.0);

    // Catch the Object
    osrf_gear::VacuumGripperControl gripper_srv;
    ros::ServiceClient client = nh_.serviceClient<osrf_gear::VacuumGripperControl>("gripper_control");
    gripper_srv.request.enable = true;
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
    if (client.call(gripper_srv))
  {
    ROS_INFO("Success");
  }

  ROS_INFO("Getting into the Home Position");
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  // sleep(2.0);

  _manipulatorgroup.setJointValueTarget(home_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(3.0);

  // drop the part
  gripper_srv.request.enable = false;
  client.call(gripper_srv);

}


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "simple_pick_and_place");
	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");
    double initialjoints[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double z_offset_from_part;

	private_node_handle.getParam("joint1", initialjoints[1]);
	private_node_handle.getParam("joint2", initialjoints[2]);
	private_node_handle.getParam("joint3", initialjoints[3]);
	private_node_handle.getParam("joint4", initialjoints[4]);
	private_node_handle.getParam("joint5", initialjoints[5]);
	private_node_handle.getParam("joint6", initialjoints[6]);
	private_node_handle.getParam("z_offset", z_offset_from_part);

    ROS_INFO_STREAM("Initial Joint 6: " << initialjoints[6] << "\n");
	PickAndPlace pickPlace(n, initialjoints, z_offset_from_part);
	pickPlace.pickNextPart();

	return 0;
}
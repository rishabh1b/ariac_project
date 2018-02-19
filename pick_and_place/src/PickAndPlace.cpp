#include "pick_and_place/PickAndPlace.h"

PickAndPlace::PickAndPlace(ros::NodeHandle nh_, double* initialjoints, double z_offset_from_part, double* part_location)
	:_manipulatorgroup("manipulator") {
	this->nh_ = nh_;
	this->_z_offset_from_part = z_offset_from_part;

	try {
    	this->tf_tray_to_world.waitForTransform("/world", "/agv2_load_point_frame", ros::Time(0), ros::Duration(20.0) );
  } catch (tf::TransformException &ex) {
    	ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
    	ros::Duration(1.0).sleep();
  }

  try {
    this->tf_tray_to_world.lookupTransform("/world", "/agv2_load_point_frame", ros::Time(0), (this->_tray_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

  _tray_location_x = _tray_to_world_.getOrigin().x();
  _tray_location_y = _tray_to_world_.getOrigin().y();
  _tray_location_z = _tray_to_world_.getOrigin().z();


  _manipulatorgroup.getCurrentState()->copyJointGroupPositions(_manipulatorgroup.getCurrentState()->getRobotModel()->getJointModelGroup(_manipulatorgroup.getName()), home_joint_values);

  // Set the client for Gripper control service
  gripper_client = nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");

  for (size_t i = 1; i < 7; i++) {
  	home_joint_values[i] = initialjoints[i];
  }

  // ROS_INFO_STREAM("Value of Each joint: " << home_joint_values[1] << " , " << home_joint_values[2] << " , " << home_joint_values[3] << " , " << home_joint_values[4] << " , " <<home_joint_values[5] << " , " << home_joint_values[6] );
  initialSetup();

  // For testing the pick and place routine
  test_x = part_location[0];
  test_y = part_location[1];
  test_z = part_location[2];

}
// Method to bring UR10 in proper position and orientation
void PickAndPlace::initialSetup() {

  goHome();
  tf::StampedTransform transform;
  tf::TransformListener listener;
  // Taking Orientation from moveit interface does not yield result on expected lines
  // Orientation changes even if same values are used
  /*geometry_msgs::PoseStamped currPose = _manipulatorgroup.getCurrentPose();
  geometry_msgs::Point currPoint = currPose.pose.position;
  _home_orientation = currPose.pose.orientation;*/

  // Use the tf tree to get the current orientation of the Wrist. 
  // Persist this orientation data for every movement
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

  // Setup the End of BaseLink Values - joint values before dropping off the part at the end of the base link
  base_link_end_values = home_joint_values;
  base_link_end_values[0] = -2.2;
  base_link_end_values[1] = 4.2;
  // return_home_joint_values = home_joint_values;
  // return_home_joint_values[1] = 0;

}

void PickAndPlace::pickNextPart(geometry_msgs::Vector3 obj_pose) {
	ROS_INFO("Picking the Next Part");
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;

	// Starting Postion before activation suction. Hardcoded for now for a single piece
  	target_pose1.position.x = obj_pose.x;
  	target_pose1.position.y = obj_pose.y;
  	target_pose1.position.z = obj_pose.z + _z_offset_from_part;
	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(3.0);

    // Catch the Object by activating the suction
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
  	ROS_INFO("Gripper Service Successfull");
   }

  // TODO: Confirm the state on the vaccum gripper before continuing
  // Wait for a bit 
  sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = 0.723 + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(3.0);
}

void PickAndPlace::pickNextPart() {
	ROS_INFO("Picking the First Part");
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;

	// Starting Postion before activation suction. Hardcoded for now for a single piece
  	target_pose1.position.x = test_x;
  	target_pose1.position.y = test_y;
  	target_pose1.position.z = test_z + _z_offset_from_part;
	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(3.0);

    // Catch the Object by activating the suction
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
  	ROS_INFO("Gripper Service Successfull");
   }

  // TODO: Confirm the state on the vaccum gripper before continuing
  // Wait for a bit 
  sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = 0.723 + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(3.0);
}

void PickAndPlace::place() {
    ROS_INFO("Placing the part");
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

	_manipulatorgroup.setJointValueTarget(base_link_end_values);
	 bool success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(5.0);

	// place it in the drop location on the AGV
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;

  	target_pose1.position.x = _tray_location_x;
  	target_pose1.position.y = _tray_location_y;
  	target_pose1.position.z = _tray_location_z + _z_offset_from_part * 5;
	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(2.0);

    // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

  // Return to the Home Position
  	_manipulatorgroup.setJointValueTarget(base_link_end_values);
	 success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(3.0);
    _manipulatorgroup.setJointValueTarget(home_joint_values);
	success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(5.0);
}

void PickAndPlace::goHome() {
  ROS_INFO("Getting into the Home Position");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  _manipulatorgroup.setJointValueTarget(home_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(3.0);

}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "simple_pick_and_place");
	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");
    double initialjoints[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double z_offset_from_part, x_test, y_test, z_test; // Some Distance offset in Z before activating suction

	private_node_handle.getParam("joint1", initialjoints[1]);
	private_node_handle.getParam("joint2", initialjoints[2]);
	private_node_handle.getParam("joint3", initialjoints[3]);
	private_node_handle.getParam("joint4", initialjoints[4]);
	private_node_handle.getParam("joint5", initialjoints[5]);
	private_node_handle.getParam("joint6", initialjoints[6]);
	private_node_handle.getParam("z_offset", z_offset_from_part);
	private_node_handle.getParam("x_test", x_test);
	private_node_handle.getParam("y_test", y_test);
	private_node_handle.getParam("z_test", z_test);

	double part_location[3] = {x_test, y_test, z_test};

    // ROS_INFO_STREAM("Initial Joint 6: " << initialjoints[6] << "\n");
	PickAndPlace pickPlace(n, initialjoints, z_offset_from_part, part_location);
	pickPlace.pickNextPart();
	pickPlace.place();

	return 0;
}
#include "pick_and_place/PickAndPlace.h"

PickAndPlace::PickAndPlace(ros::NodeHandle nh_, double* initialjoints, double z_offset_from_part, double* part_location, float tray_length)
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


  _manipulatorgroup.getCurrentState()->copyJointGroupPositions(_manipulatorgroup.getCurrentState()->getRobotModel()->getJointModelGroup(_manipulatorgroup.getName()), home_joint_values);

  // Set the client for Gripper control service
  gripper_client = nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");

  // Set the Gripper State Subsrciber
  gripperStateSubscriber = nh_.subscribe("/ariac/gripper/state", 10, &PickAndPlace::gripperStateCallback, this);

  for (size_t i = 1; i < 7; i++) {
  	home_joint_values[i] = initialjoints[i];
  }

  // ROS_INFO_STREAM("Value of Each joint: " << home_joint_values[1] << " , " << home_joint_values[2] << " , " << home_joint_values[3] << " , " << home_joint_values[4] << " , " <<home_joint_values[5] << " , " << home_joint_values[6] );
  initialSetup();

  // For testing the pick and place routine
  test_x = part_location[0];
  test_y = part_location[1];
  test_z = part_location[2];

   // Assumed tray length
  _tray_length = tray_length;

   srand (static_cast <unsigned> (time(0)));

   _isPartAttached = false;
   _nowExecuting = false;

   index = 0;

   conveyor_joint_values = home_joint_values;
   conveyor_joint_values[1] = 6.27;
   // Set Planning Time
   // _manipulatorgroup.setPlanningTime(1);

}
// Method to bring UR10 in proper position and orientation
void PickAndPlace::initialSetup() {

  goHome();
  tf::StampedTransform transform;
  tf::StampedTransform worldtransform;
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

  listener.waitForTransform("world","ee_link",ros::Time(0), ros::Duration(2));
      try{
        listener.lookupTransform("/world", "ee_link",  
                    ros::Time(0), worldtransform);
       }
      catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
  }

  _home_quat = transform.getRotation().normalize();

  // _home_orientation.x = transform.getRotation().x();
  // _home_orientation.y = transform.getRotation().y();
  // _home_orientation.z = transform.getRotation().z();
  // _home_orientation.w = transform.getRotation().w();

  _home_orientation.x = _home_quat.x();
  _home_orientation.y = _home_quat.y();
  _home_orientation.z = _home_quat.z();
  _home_orientation.w = _home_quat.w();

  _home_position.x = worldtransform.getOrigin().getX();
  _home_position.y = worldtransform.getOrigin().getY();
  _home_position.z = worldtransform.getOrigin().getZ();

  geometry_msgs::Point _home_pt;
  _home_pt.x = _home_position.x;
  _home_pt.y = _home_position.y;
  _home_pt.z = _home_position.z;

  _homePose.position = _home_pt;
  _homePose.orientation = _home_orientation;

  tf::Quaternion quat(_home_orientation.x, _home_orientation.y, _home_orientation.z, _home_orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM("Roll : " << roll);
  ROS_INFO_STREAM("Pitch : " << pitch);
  ROS_INFO_STREAM("yaw : " << yaw);

  // Try a different Home Orientation
  tf::Quaternion q  = tf::createQuaternionFromRPY(0, 1.57, 3.14); 
  // tf::Quaternion q = tf::createQuaternionFromRPY(roll, 1.57, yaw);
  _home_orientation.x = q.x();
  _home_orientation.y = q.y();
  _home_orientation.z = q.z();
  _home_orientation.w = q.w();

  // Setup the End of BaseLink Values - joint values before dropping off the part at the end of the base link
  base_link_end_values = home_joint_values;
  base_link_end_values[0] = -2;
  base_link_end_values[1] = 4.69;
  base_link_end_values[2] = base_link_end_values[2] + 0.7;
  base_link_end_values[3] = base_link_end_values[3] - 0.7;

  base_link_end_values_2 = home_joint_values;
  base_link_end_values_2[0] = 2;
  base_link_end_values_2[1] = 1.56;
  base_link_end_values_2[2] = base_link_end_values[2] - 0.7;
  base_link_end_values_2[3] = base_link_end_values[3] + 0.7;

  // Position on the belt to pick from - Static Position for now
  conveyor_x = 1.21;
  conveyor_y = 1.77;
  conveyor_z = 0.93;

  ROS_INFO_STREAM("Home Position X: " << _home_position.x);
  ROS_INFO_STREAM("Home Position Y: " << _home_position.y);
  ROS_INFO_STREAM("Home Position Z: " << _home_position.z);

}

bool PickAndPlace::pickNextPart(geometry_msgs::Vector3 obj_pose, geometry_msgs::Quaternion orientation) {
  ROS_INFO("Picking the Next Part");
  ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;
  // target_pose1.orientation = orientation;

  // Starting Postion before activation suction. Hardcoded for now for a single piece
    target_pose1.position.x = obj_pose.x;
    target_pose1.position.y = obj_pose.y;
    target_pose1.position.z = obj_pose.z + _z_offset_from_part;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
    sleep(1.0);

    // Catch the Object by activating the suction
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
    ROS_INFO("Gripper Service Successfull");
   }
 
  // Wait for a bit 
  ros::spinOnce();
  sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = 0.723 + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(1.0);

  return _isPartAttached;
}

bool PickAndPlace::pickPlaceNextPartConveyor(geometry_msgs::Vector3 obj_pose, geometry_msgs::Vector3 target_pose, 
                                            geometry_msgs::Quaternion target_orientation, bool useAGV2) {
  ROS_INFO("Picking the Next Part");
  ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

  _manipulatorgroup.setJointValueTarget(conveyor_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1.0);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;
  // target_pose1.orientation = orientation;

  // Starting Postion before activation suction. Hardcoded for now for a single piece
    target_pose1.position.x = obj_pose.x;
    target_pose1.position.y = obj_pose.y;
    target_pose1.position.z = obj_pose.z + 0.82 * _z_offset_from_part;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
    sleep(1.0);

    // Catch the Object by activating the suction
    // Introduce a little delay
    // Make it robust by calculating the exact time based on velocity
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
    ROS_INFO("Gripper Service Successfull");
   }
   // sleep(7);
 
  // Wait for a bit 
  ros::spinOnce();
  sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = conveyor_z + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(1.0);

  if (!_isPartAttached)
    return false;

  if (!useAGV2)
    _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
  else
    _manipulatorgroup.setJointValueTarget(base_link_end_values);

  success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
    sleep(1.0);
    
 if(!_isPartAttached)
    return false;
 // waypoints.push_back(target_pose2);

 target_pose1.position.x = target_pose.x;
 target_pose1.position.y = target_pose.y;
 target_pose1.position.z = target_pose.z + _z_offset_from_part * 5;
 _manipulatorgroup.setPoseTarget(target_pose1);
 _manipulatorgroup.move();
  sleep(1.0);
  if(!_isPartAttached)
    return false;

  sleep(1.0);
  ros::spinOnce();
  sleep(1.0);

 if (!_isPartAttached)
    return false;

 // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

 // Return to the Home Position
    if (!useAGV2) {
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 1.2;
    target_pose1.position.z = 1.3;
   } else {
    target_pose1.position.x = 0.3;
    target_pose1.position.y = -1.2;
    target_pose1.position.z = 1.3;
   }
   
    _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(1.0);
    _manipulatorgroup.setJointValueTarget(home_joint_values);
    success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    index++;
    return true;
}


bool PickAndPlace::pickNextPart(geometry_msgs::Vector3 obj_pose) {
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
    sleep(1.0);

    // Catch the Object by activating the suction
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
  	ROS_INFO("Gripper Service Successfull");
   }

  // Wait for a bit 
  ros::spinOnce();
  sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = 0.723 + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(1.0);
  return _isPartAttached;

}

void PickAndPlace::pickNextPart() {
  double t1 = ros::Time::now().toSec();
	ROS_INFO("Picking the Next Part");
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

    _manipulatorgroup.setJointValueTarget(conveyor_joint_values);
    bool success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);

  double t2 = ros::Time::now().toSec();
  ROS_INFO_STREAM("The in Place Rotation Time is: " << t2-t1);
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;

	// Starting Postion before activation suction. Hardcoded for now for a single piece
    t1 = ros::Time::now().toSec();
  	target_pose1.position.x = test_x;
  	target_pose1.position.y = test_y;
  	target_pose1.position.z = test_z + _z_offset_from_part;
	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(1.0);
    t2 = ros::Time::now().toSec();
    ROS_INFO_STREAM("The moving to the end of beam time is: " << t2-t1);
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
   sleep(1.0);

  goHome();
  goHome2();
}

bool PickAndPlace::place() {
   float position_x[5] = {0, -0.1, 0, 0.1, 0};
   float position_y[5] = {0, 0.1, 0.1, 0, 0.25};

    ROS_INFO("Placing the part now");
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

	_manipulatorgroup.setJointValueTarget(base_link_end_values);
	 bool success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(3.0);
    ros::spinOnce();
    sleep(3.0);
    if (!_isPartAttached)
  	  return false; 

	// place it in the drop location on the AGV
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;

    // ROS_INFO_STREAM("Random Value Obtained: " << getRandomValue());
    target_pose1.position.x = _tray_location_x + position_x[index];
  	target_pose1.position.y = _tray_location_y + position_y[index];
  	// target_pose1.position.x = _tray_location_x + getRandomValue();
  	// target_pose1.position.y = _tray_location_y + getRandomValue();
  	target_pose1.position.z = _tray_location_z + _z_offset_from_part * 5;

	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(3.0);

    if (!_isPartAttached)
  	  return false; 

    // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

  // Return to the Home Position
  	_manipulatorgroup.setJointValueTarget(base_link_end_values);
	 success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    _manipulatorgroup.setJointValueTarget(home_joint_values);
	success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    index++;
    return true;
}

bool PickAndPlace::place(geometry_msgs::Vector3 vec, geometry_msgs::Quaternion quat) {
   float position_x[5] = {0, -0.1, 0, 0.1, 0};
   float position_y[5] = {0, 0.1, 0.1, 0, 0.25};

    ROS_INFO("Placing the part now");
  ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

  _manipulatorgroup.setJointValueTarget(base_link_end_values);
   bool success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    ros::spinOnce();
    sleep(1.0);
    if (!_isPartAttached)
      return false; 

  // place it in the drop location on the AGV
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;

    target_pose1.position.x = vec.x;
    target_pose1.position.y = vec.y;
    // target_pose1.position.x = _tray_location_x + getRandomValue();
    // target_pose1.position.y = _tray_location_y + getRandomValue();
    target_pose1.position.z = vec.z + _z_offset_from_part * 5;
   _manipulatorgroup.setPoseTarget(target_pose1);
   // ROS_INFO_STREAM("Get Position Tolerance: " << _manipulatorgroup.getGoalPositionTolerance());
   // ROS_INFO_STREAM("Get Orientation Tolerance: " << _manipulatorgroup.getGoalOrientationTolerance());
   // _manipulatorgroup.setGoalPositionTolerance(0.1);
   // _manipulatorgroup.setGoalOrientationTolerance(0.01);

  _manipulatorgroup.move();
    sleep(3.0);

    if (!_isPartAttached)
      return false; 

    // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

  // Return to the Home Position
    _manipulatorgroup.setJointValueTarget(base_link_end_values);
   success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    _manipulatorgroup.setJointValueTarget(home_joint_values);
    success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    index++;
    return true;
}

bool PickAndPlace::pickAndPlace(geometry_msgs::Vector3 obj_pose, geometry_msgs::Quaternion obj_orientation, geometry_msgs::Vector3 target_pose, 
                                geometry_msgs::Quaternion target_orientation, bool useAGV2) {

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
    sleep(2.0);

    // Catch the Object by activating the suction
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
    ROS_INFO("Gripper Service Successfull");
   }

  // Wait for a bit 
  ros::spinOnce();
  sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = 0.723 + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(1.0);

  if(!_isPartAttached)
    return false;

  // _manipulatorgroup.setJointValueTarget(home_joint_values);
  // bool success = _manipulatorgroup.plan(my_plan);
  // _manipulatorgroup.move();
  // sleep(1.0); 

  _nowExecuting = true;
  goHome();
  std::vector<geometry_msgs::Pose> waypoints;

 geometry_msgs::Pose target_pose2;
 if (!useAGV2) {
   target_pose2.position.x = 0.3; //0.3
   target_pose2.position.y = 1.2; // -1.2
   target_pose2.position.z = 1.3; // 1.3
 } else {
   target_pose2.position.x = 0.3; //0.3
   target_pose2.position.y = -1.2; // -1.2
   target_pose2.position.z = 1.3; // 1.3
 }

 target_pose2.orientation = _home_orientation;
 _manipulatorgroup.setPoseTarget(target_pose2);
 _manipulatorgroup.move();
  sleep(1.0);
  if(!_isPartAttached)
    return false;

  if (!useAGV2)
    _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
  else
    _manipulatorgroup.setJointValueTarget(base_link_end_values);

  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
    sleep(1.0);
    
 if(!_isPartAttached)
    return false;
 // waypoints.push_back(target_pose2);

 target_pose2.position.x = target_pose.x;
 target_pose2.position.y = target_pose.y;
 target_pose2.position.z = target_pose.z + _z_offset_from_part * 5;
 _manipulatorgroup.setPoseTarget(target_pose2);
 _manipulatorgroup.move();
  sleep(1.0);
  if(!_isPartAttached)
    return false;

_nowExecuting = false;

 sleep(1.0);
 ros::spinOnce();
 sleep(1.0);

 if (!_isPartAttached)
    return false;

 // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

 // Return to the Home Position
    if (!useAGV2) {
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 1.2;
    target_pose1.position.z = 1.3;
   } else {
    target_pose1.position.x = 0.3;
    target_pose1.position.y = -1.2;
    target_pose1.position.z = 1.3;
   }
   
    _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(1.0);
    _manipulatorgroup.setJointValueTarget(home_joint_values);
    success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    index++;
    return true;

}

void PickAndPlace::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr& msg) {
	_isPartAttached = msg->attached;
  // if (!_isPartAttached && _nowExecuting)
  //   _manipulatorgroup.stop();
}

void PickAndPlace::goHome() {
  ROS_INFO("Getting into the Home Position");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  _manipulatorgroup.setJointValueTarget(home_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1.0);

}

void PickAndPlace::goHome2() {
  ROS_INFO("Getting into the Home Position");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  std::vector<double> home_joint_values2 = home_joint_values;
  home_joint_values2[6] = 3.14;
  _manipulatorgroup.setJointValueTarget(home_joint_values2);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1.0);

}

float PickAndPlace::getRandomValue() {
	// ROS_WARN("Tray Length is: %f", _tray_length);
	return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/_tray_length));
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "simple_pick_and_place");
	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");
    double initialjoints[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Some Distance offset in Z before activating suction
    double z_offset_from_part, x_test, y_test, z_test; 
    float tray_length;

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
	private_node_handle.getParam("tray_length", tray_length);

	double part_location[3] = {x_test, y_test, z_test};

    // ROS_INFO_STREAM("Initial Joint 6: " << initialjoints[6] << "\n");
	PickAndPlace pickPlace(n, initialjoints, z_offset_from_part, part_location, tray_length);
	pickPlace.pickNextPart();
	// pickPlace.place();

	return 0;
}

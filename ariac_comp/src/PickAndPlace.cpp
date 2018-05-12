#include "ariac_comp/PickAndPlace.h"

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

  for (size_t i = 1; i < 7; i++) {
    home_joint_values[i] = initialjoints[i];
  }

  // Set the client for Gripper control service
  gripper_client = nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");

  // Set the Gripper State Subsrciber
  gripperStateSubscriber = nh_.subscribe("/ariac/gripper/state", 10, &PickAndPlace::gripperStateCallback, this);

  // Set the Quality Sensor Subscriber - 
  qual_subscriber_1 = nh_.subscribe("/ariac/quality_control_sensor_1", 10, &PickAndPlace::qualSensor1, this);
  qual_subscriber_2 = nh_.subscribe("/ariac/quality_control_sensor_2", 10, &PickAndPlace::qualSensor2, this);

  initialSetup();

  // For testing the pick and place routine
  test_x = part_location[0];
  test_y = part_location[1];
  test_z = part_location[2];


   srand (static_cast <unsigned> (time(0)));

   _isPartAttached = false;
   _nowExecuting = false;

   index = 0;

   conveyor_joint_values = home_joint_values;
   conveyor_joint_values[1] = 0.1;
   _conveyorPartPicked = true;

   // Set Planning Time
   _manipulatorgroup.setPlanningTime(10);

   // Some Offsets that I know work well for conveyor picking
   conveyorPickOffset.insert(std::make_pair("gasket_part", 0.5));
   conveyorPickOffset.insert(std::make_pair("disk_part", 0.6));
   conveyorPickOffset.insert(std::make_pair("piston_rod_part", 0.05));
   conveyorPickOffset.insert(std::make_pair("gear_part", 0.08));

  _isfaultyAGV2 = false;
  _isfaultyAGV1 = false;
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

  _tray_orientation.x = _home_quat.x();
  _tray_orientation.y = _home_quat.y();
  _tray_orientation.z = _home_quat.z();
  _tray_orientation.w = _home_quat.w();

  _home_position.x = worldtransform.getOrigin().getX();
  _home_position.y = worldtransform.getOrigin().getY();
  _home_position.z = worldtransform.getOrigin().getZ();

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

  // Try a different Home Orientation - which is more parallel
  tf::Quaternion q  = tf::createQuaternionFromRPY(0, 1.57, 3.14); 
  _home_orientation.x = q.x();
  _home_orientation.y = q.y();
  _home_orientation.z = q.z();
  _home_orientation.w = q.w();

  // setHome();

  // Setup the End of BaseLink Values - joint values before dropping off the part at the end of the base link
  base_link_end_values = home_joint_values;
  base_link_end_values[0] = -2.2;
  base_link_end_values[1] = 4.719; // -1.5
  base_link_end_values[2] = base_link_end_values[2] + 0.5;
  base_link_end_values[3] = base_link_end_values[3] - 0.5;

  base_link_end_values_2 = home_joint_values;
  base_link_end_values_2[0] = 2.2;
  base_link_end_values_2[1] = 1.56; // 1.56
  // base_link_end_values_2[2] = base_link_end_values_2[2] - 0.8;
  // base_link_end_values_2[3] = base_link_end_values_2[3] + 0.8;

  // Setup Dumping joint values
  dump_joint_values = home_joint_values;
  dump_joint_values[0] = -2.2;
  dump_joint_values[1] = 3.9325; // -1.5
  dump_joint_values[2] = dump_joint_values[2] + 0.5;
  dump_joint_values[3] = dump_joint_values[3] - 0.5;

  dump_joint_values_2 = home_joint_values;
  dump_joint_values_2[0] = 2.2;
  dump_joint_values_2[1] = 2.53; // 1.56 

  // Scanning Part joint values
  scan_joint_values = home_joint_values;
  scan_joint_values[0] = 1.5;
  scan_joint_values[1] = 6.27; // 0.785 

  // scan_joint_values_2 = home_joint_values;
  // scan_joint_values_2[0] = 1.5;
  // scan_joint_values_2[1] = 6.27; 

  // std::vector<double> conveyor_scan_joint_values;
  double pose[] = {2.1, 6.27, -0.988, 1.64, 4.0712, 4.7124, -3.1};//{1.64, 0.0398, -0.988, 0.0398, 4.0712, 4.7124, -3.1};
  conveyor_scan_joint_values.clear();
  for(size_t i = 0; i < 7;i++){
    conveyor_scan_joint_values.push_back(pose[i]);
  }

  // Position on the belt to pick from - Some dummy values, we will used conveyor_z
  conveyor_x = 1.21;
  conveyor_y = 1.77;
  conveyor_z = 0.93;

  // Setup Part Location
  FactoryFloor ff(nh_);
  partLocation = ff.getPartLocations();

  // Setup Bins
  // Bin bin5("bin5", -0.1, -0.741 - 0.765, 0.72);
  // Bin bin6("bin6", -0.1, -0.741, 0.72);
  // Bin bin7("bin7", -0.1, -0.741 + 0.765, 0.72);
  // Bin bin8("bin8", -0.1, -0.741 + 1.53, 0.72);

  Bin bin5("bin5", -0.1, -1.53, 0.72);
  Bin bin6("bin6", -0.1, -0.741, 0.72);
  Bin bin7("bin7", -0.1, -0.741 + 0.765, 0.72);
  Bin bin8("bin8", -0.1, -0.741 + 1.53, 0.72);

  binMap.insert(std::make_pair("bin5", bin5));
  binMap.insert(std::make_pair("bin6", bin6));
  binMap.insert(std::make_pair("bin7", bin7));
  binMap.insert(std::make_pair("bin8", bin8));

  // Check whether map is properly populated
  std::map<std::string, std::string>::iterator it = partLocation.begin();

  if (partLocation.size() == 0) {
    ROS_WARN("No Parts in Bin");
  }

  else {
  ROS_WARN("Printing PartLocation Map");
  while (it != partLocation.end()) {
    std::cout << " For Part Type " << it->first << " the location is " << it->second << std::endl;
    if (it->first == "piston_rod_part"){
      binMap[it->second].z_offset_from_part = _z_offset_from_part ;
      binMap[it->second].setResolution(0.125);
    }
    else if (it->first == "gear_part"){
      binMap[it->second].z_offset_from_part = _z_offset_from_part * 1.4;
      binMap[it->second].setResolution(0.1); // 0.133
    }
    else if (it->first == "disk_part"){
      binMap[it->second].z_offset_from_part = _z_offset_from_part * 2;
      binMap[it->second].setResolution(0.1);
    }
    else if (it->first == "gasket_part") {
      binMap[it->second].z_offset_from_part = _z_offset_from_part * 1.8;
      binMap[it->second].setResolution(0.1);
    }
    else if (it->first == "pulley_part"){
      binMap[it->second].z_offset_from_part = _z_offset_from_part * 4.5;
      binMap[it->second].setResolution(0.3);
    }
    ++it;
  }
}

}

void PickAndPlace::attainConveyorPick(bool useAGV2) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

  if (!useAGV2) {
     conveyor_joint_values[0] = base_link_end_values_2[0];
     conveyor_joint_values[1] = 0.01;
  }
  else {
     conveyor_joint_values[0] = base_link_end_values[0];
     conveyor_joint_values[1] = 6.27;
  } 

  _manipulatorgroup.setJointValueTarget(conveyor_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1);
}

void PickAndPlace::goToJointValue(double linear_joint) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(0.1);
  std::vector<double> joint_vals = home_joint_values;
  joint_vals[0] = linear_joint;
  _manipulatorgroup.setJointValueTarget(joint_vals);

  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1.0);
}

void PickAndPlace::goToScanLocation(bool conveyorPicking) {
  ROS_INFO("Going to Scan the part now!");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(0.1);

  bool success;
 //  if (!conveyorPicking) {
 //    _manipulatorgroup.setJointValueTarget(scan_joint_values);

 //    success = _manipulatorgroup.plan(my_plan);
 //    _manipulatorgroup.move();
 //    sleep(1.5);
 // }

 // else {
 //    _manipulatorgroup.setJointValueTarget(scan_joint_values_2);

 //    success = _manipulatorgroup.plan(my_plan);
 //    _manipulatorgroup.move();
 //    sleep(1.5);
 // }

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;

  // Scanning Location for Part
  // target_pose1.position.x = 1.2;
  // target_pose1.position.y = 2.3;
  // target_pose1.position.z = 1.2;
  // _manipulatorgroup.setPoseTarget(target_pose1);
  // success = _manipulatorgroup.plan(my_plan);

  _manipulatorgroup.setJointValueTarget(conveyor_scan_joint_values);
  bool success1 = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1);

  int counter = 5;

  // Should go away.
  while (!success && counter > 0) {
    success = _manipulatorgroup.plan(my_plan);
    --counter;
  }
  _manipulatorgroup.move();
    sleep(1.0);

  // Get the Transform
  tf::StampedTransform transform;
  tf::TransformListener listener; 

  // Use the tf tree to get the current orientation of the Wrist. 
  listener.waitForTransform("/world","ee_link",ros::Time(0), ros::Duration(2));
    try{
        listener.lookupTransform("/world", "ee_link",  
                    ros::Time(0), transform);
       }
      catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
  }

  _offset_vect = transform.getOrigin();
}

bool PickAndPlace::pickNextPartBin(std::string partType, bool& partAvailable, bool& partNotInReach) {

  if (partLocation.find(partType) == partLocation.end()) {
    partAvailable = false;
    return true;
  }

  partAvailable = true;

  ROS_INFO("Picking the Next Part From Bin");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(0.1);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;

  _curr_part_type = partType;

  // Turn the Gripper onby activating the suction
  gripper_srv.request.enable = true;

  if (gripper_client.call(gripper_srv))
  {
    ROS_INFO("Gripper Service Called");
  }
 
  target_pose1.position.x = binMap[partLocation[partType]].start_x;
  target_pose1.position.y = binMap[partLocation[partType]].start_y;

  goToJointValue(target_pose1.position.y);
  // A Logic in a Loop which tries to find the part blindly based on the bin location 
  double linear_joint_val = 0;
  int i;
  for (i = 0; i < 20; i++) { // At Max Twenty Trials
     target_pose1.position.z = binMap[partLocation[partType]].start_z + binMap[partLocation[partType]].z_offset_from_part;
    _manipulatorgroup.setPoseTarget(target_pose1);
    _manipulatorgroup.move();
     sleep(3);

    // Lift the arm a little up
    ros::spinOnce();
    target_pose1.position.z = binMap[partLocation[partType]].start_z + binMap[partLocation[partType]].z_offset_from_part * 4;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(0.1);
    ros::spinOnce();

    if (_isPartAttached) {
      // binMap[partLocation[partType]].start_z = binMap[partLocation[partType]].start_z + binMap[partLocation[partType]].z_offset_from_part * 4;
      binMap[partLocation[partType]].setPartFound();
      linear_joint_val = binMap[partLocation[partType]].start_y;
      break;
    }

     binMap[partLocation[partType]].incStep();
     target_pose1.position.x = binMap[partLocation[partType]].start_x;
     target_pose1.position.y = binMap[partLocation[partType]].start_y;
    _manipulatorgroup.setPoseTarget(target_pose1);
    _manipulatorgroup.move();
     sleep(0.1);
     ros::spinOnce();
  }

  // if (i == 3) // change searching strategy
  goToJointValue(linear_joint_val);
  if (i == 20)
    partNotInReach = true;

  if (!_isPartAttached)
    return false;

  return true;
} 

bool PickAndPlace::isPartAttached() {
  return _isPartAttached;
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

bool PickAndPlace::pickNextPartConveyor(geometry_msgs::Vector3 obj_pose, geometry_msgs::Vector3 target_pose, geometry_msgs::Quaternion target_orientation,
                                        std::string partType, bool useAGV2) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(0.1);

  if (!useAGV2) {
     conveyor_joint_values[0] = base_link_end_values_2[0];
     conveyor_joint_values[1] = 0.01;
     obj_pose.y = 2.1 - obj_pose.y;
  }
 else {
     conveyor_joint_values[0] = base_link_end_values[0];
     conveyor_joint_values[1] = 6.27;
     obj_pose.y = obj_pose.y - 2.1;
 } 


  ROS_INFO_STREAM(" Picking Location on Conveyor : " << obj_pose.y);

  _manipulatorgroup.setJointValueTarget(conveyor_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1);

  gripper_srv.request.enable = true;

  if (gripper_client.call(gripper_srv))
  {
    ROS_INFO("Gripper Service Called conveyor");
  }

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;

  ROS_INFO_STREAM(" The offset is : " << conveyorPickOffset[partType]);
  target_pose1.position.x = obj_pose.x;
  target_pose1.position.y = obj_pose.y;
  target_pose1.position.z = obj_pose.z + conveyorPickOffset[partType] * _z_offset_from_part;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
  sleep(2);

  double t = ros::Time::now().toSec();  
  while (!_isPartAttached) {
    if (ros::Time::now().toSec() - t > 2)
        break;
    ros::spinOnce();
    target_pose1.position.z = conveyor_z + _z_offset_from_part * conveyorPickOffset[partType] * 8;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(0.1);
   ros::spinOnce();
   target_pose1.position.z = obj_pose.z + conveyorPickOffset[partType] * _z_offset_from_part;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
    sleep(0.1);

}
  if (!_isPartAttached)
    return false;
  else {
    target_pose1.position.z = conveyor_z + _z_offset_from_part * conveyorPickOffset[partType] * 18;
    _manipulatorgroup.setPoseTarget(target_pose1);
    _manipulatorgroup.move();
    sleep(0.1);
    return true;
  }
}

bool PickAndPlace::pickPlaceNextPartConveyor(geometry_msgs::Vector3 obj_pose, geometry_msgs::Vector3 target_pose, 
                                            geometry_msgs::Quaternion target_orientation, bool useAGV2) {
  ROS_INFO("Picking the Next Part");
  ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

  if (!useAGV2) {
       conveyor_joint_values[0] = base_link_end_values_2[0];
       conveyor_joint_values[1] = 0.01;
     }
     else {
       conveyor_joint_values[0] = base_link_end_values[0];
       conveyor_joint_values[1] = 6.27;
     }

  _manipulatorgroup.setJointValueTarget(conveyor_joint_values);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1);


  // Catch the Object by activating the suction
  // Introduce a little delay
  // Make it robust by calculating the exact time based on velocity
    gripper_srv.request.enable = true;

    if (gripper_client.call(gripper_srv))
   {
    ROS_INFO("Gripper Service Successfull");
   }

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;
  // target_pose1.orientation = orientation;

    target_pose1.position.x = obj_pose.x;
    target_pose1.position.y = obj_pose.y;
    target_pose1.position.z = obj_pose.z + 0.6 * _z_offset_from_part;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
    sleep(0.1);


  // Wait for a bit 
   sleep(2);
   target_pose1.position.z = conveyor_z + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(0.5);

  double t = ros::Time::now().toSec();
  while (!_isPartAttached) {
    if (ros::Time::now().toSec() - t > 3)
        break;
    ros::spinOnce();
    // sleep(3.0);

  // Lift the arm a little up
  target_pose1.position.z = conveyor_z + _z_offset_from_part * 5;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
   sleep(0.1);
   ros::spinOnce();
   target_pose1.position.z = obj_pose.z + 0.6 * _z_offset_from_part;
  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
    sleep(0.1);
 }

  if (!_isPartAttached)
    return false;

  if (!useAGV2)
    _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
  else
    _manipulatorgroup.setJointValueTarget(base_link_end_values);

  success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(2.0);
    
 if(!_isPartAttached)
    return false;

 target_pose1.position.x = target_pose.x;
 target_pose1.position.y = target_pose.y;
 target_pose1.position.z = target_pose.z + _z_offset_from_part * 11;
 _manipulatorgroup.setPoseTarget(target_pose1);
 _manipulatorgroup.move();
  sleep(1.0);
  if(!_isPartAttached)
    return false;

  sleep(1.0);
  ros::spinOnce();

 if (!_isPartAttached)
    return false;

 // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

    _conveyorPartPicked = true;
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
  goHome();
  double t1 = ros::Time::now().toSec();
	ROS_INFO("Picking the Next Part");
	ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

    // _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
    // bool success = _manipulatorgroup.plan(my_plan);
    // _manipulatorgroup.move();
    // sleep(1.0);

 //  double t2 = ros::Time::now().toSec();
 //  ROS_INFO_STREAM("The in Place Rotation Time is: " << t2-t1);
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = _home_orientation;

 //    t1 = ros::Time::now().toSec();
 //  	target_pose1.position.x = test_x - 0.06;
 //  	target_pose1.position.y = test_y + 0.06;
 //  	target_pose1.position.z = test_z + _z_offset_from_part * 1.5;
	// _manipulatorgroup.setPoseTarget(target_pose1);
  target_pose1.position.x = -0.093;
  target_pose1.position.y = -0.741;
  target_pose1.position.z = test_z + _z_offset_from_part * 1.5;
  _manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();

  for (int i = 0; i < 4; i++) {
    // Lift the arm a little up
    target_pose1.position.z = test_z + _z_offset_from_part * 5;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(0.1);
    target_pose1.position.x = target_pose1.position.x - 0.1;
    target_pose1.position.y = target_pose1.position.y + 0.1;
    target_pose1.position.z = test_z + _z_offset_from_part * 5;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(0.1);
    target_pose1.position.z = test_z + _z_offset_from_part;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(0.1);
    target_pose1.position.z = test_z + _z_offset_from_part * 1.5;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
  }
 //    sleep(1.0);
 //    t2 = ros::Time::now().toSec();
 //    ROS_INFO_STREAM("The moving to the end of beam time is: " << t2-t1);
 // //    // Catch the Object by activating the suction
 // //    gripper_srv.request.enable = true;

 // //    if (gripper_client.call(gripper_srv))
 // //   {
 // //  	ROS_INFO("Gripper Service Successfull");
 // //   }

 //  sleep(3.0);

  // Lift the arm a little up
  // target_pose1.position.z = 0.723 + _z_offset_from_part * 5;
  // _manipulatorgroup.setPoseTarget(target_pose1);
  // _manipulatorgroup.move();
  //  sleep(1.0);

 //  std::vector<double> temp_joint_values = base_link_end_values_2;
 //  temp_joint_values[2] = temp_joint_values[2] + M_PI / 3;
 //  temp_joint_values[3] = temp_joint_values[3] - M_PI / 2.5;
 //  _manipulatorgroup.setJointValueTarget(temp_joint_values);
 // _manipulatorgroup.move();
 //  sleep(1.0);

  // _manipulatorgroup.setPlanningTime(10);
 //  geometry_msgs::Pose target_pose2;
 //  target_pose2.orientation = _home_orientation;
 //  target_pose2.position.x = _tray_1_x;
 // target_pose2.position.y = _tray_1_y;
 // target_pose2.position.z = _tray_1_z;
 // _manipulatorgroup.setPoseTarget(target_pose2);
 // _manipulatorgroup.move();
 //  sleep(1.0);
  // goHome();
  // goHome2();
}

bool PickAndPlace::place() {
    ROS_INFO("Placing the part now");
	  ros::AsyncSpinner spinner(1);
  	spinner.start();
  	sleep(2.0);

   geometry_msgs::Pose target_pose1;
	 _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
	 bool success = _manipulatorgroup.plan(my_plan);
   _manipulatorgroup.move();
   sleep(0.1);
   ros::spinOnce();
   sleep(0.1);

   target_pose1.orientation = _home_orientation;
   target_pose1.position.x = 0.3;
   target_pose1.position.y = 3.15;
   target_pose1.position.z = 0.751 + _z_offset_from_part * 10;

  _manipulatorgroup.setPoseTarget(target_pose1);
  _manipulatorgroup.move();
    sleep(0.1);

    if (!_isPartAttached)
  	  return false; 

	// place it in the drop location on the AGV
   tf::Quaternion quat(0, 0, 0, 1);
   double roll, pitch, yaw_t, yaw_c;
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_t);
   ROS_INFO_STREAM("yaw Target: " << yaw_t);

   tf::Quaternion quat2(_home_orientation.x, _home_orientation.y, _home_orientation.z, _home_orientation.w);
   tf::Matrix3x3(quat2).getRPY(roll, pitch, yaw_c);
   ROS_INFO_STREAM("yaw current: " << yaw_c);
   quat2 = tf::createQuaternionFromRPY(roll, pitch, -0.862438); 
   // quat2 = tf::createQuaternionFromRPY(roll, pitch, yaw_t);

	 target_pose1.orientation.x = quat2.x();
   target_pose1.orientation.y = quat2.y();
   target_pose1.orientation.z = quat2.z();
   target_pose1.orientation.w = quat2.w(); 
   target_pose1.position.x = 0.4;
   target_pose1.position.y = 2.95;
   target_pose1.position.z = 0.751 + _z_offset_from_part * 5;

	_manipulatorgroup.setPoseTarget(target_pose1);
	_manipulatorgroup.move();
    sleep(1.0);

    if (!_isPartAttached)
  	  return false; 

    // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);

  // Return to the Home Position
  	_manipulatorgroup.setJointValueTarget(base_link_end_values_2);
	   success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
    goHome();
    return true;
}

bool PickAndPlace::place(geometry_msgs::Vector3 vec_s, geometry_msgs::Quaternion quat_s, geometry_msgs::Vector3 vec_t, geometry_msgs::Quaternion quat_t, bool useAGV2) {
    ROS_INFO("Placing the part now");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

   if (!useAGV2)
    _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
  else
    _manipulatorgroup.setJointValueTarget(base_link_end_values);

   bool success = _manipulatorgroup.plan(my_plan);
   _manipulatorgroup.move();

   sleep(2);

   ros::spinOnce();

   geometry_msgs::Pose target_pose1;
   target_pose1.orientation = _home_orientation;

    // if (!_isPartAttached)
    //   return false; 

  // place it in the drop location on the AGV
  tf::Quaternion quat(quat_t.x, quat_t.y, quat_t.z, quat_t.w);
  // tf::Quaternion quat(0, 0, 0, 1);
   double roll, pitch, yaw_pt, yaw_p, yaw_e;
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_pt);
   ROS_INFO_STREAM("yaw Target: " << yaw_pt);

   // Hack
   // yaw_t = 0;

   // Calculate the Offset
   double err_x = _offset_vect.getX() - vec_s.x;
   double err_y = _offset_vect.getY() - vec_s.y;
   double err_z = _offset_vect.getZ() - vec_s.z;

   ROS_INFO_STREAM("The Error in X is : " << err_x);
   ROS_INFO_STREAM("The Error in Y is : " << err_y);
   ROS_INFO_STREAM("The Error in Z is : " << err_z);

   target_pose1.position.x = vec_t.x + err_x;
   target_pose1.position.y = vec_t.y + err_y;
   target_pose1.position.z = vec_t.z + _z_offset_from_part * 12;

    // Hack
   if (target_pose1.position.y > 3.335) {
     ROS_WARN(" For some reason y position is more than 3.335");
     target_pose1.position.y = 3.3;
   }

   if (target_pose1.position.y < -3.335)
     target_pose1.position.y = -3.3;

  _manipulatorgroup.setPoseTarget(target_pose1);
   success = _manipulatorgroup.plan(my_plan);

   int counter = 10;
   while (!success && counter > 0) {
       ROS_WARN("Plan not found for target pose drop location");
      success = _manipulatorgroup.plan(my_plan);
      --counter;
   }

  _manipulatorgroup.move();
    sleep(1.0);

  // Logic to correct the Place Location after Dropping in tray
   if (_isPartAttached && (std::abs(err_x) > 0.01 || std::abs(err_y) > 0.01) && (_curr_part_type == "gear_part")) {
    // drop the part
   gripper_srv.request.enable = false;
   gripper_client.call(gripper_srv);

   // Pick the Part again
   target_pose1.position.x = vec_t.x;
   target_pose1.position.y = vec_t.y;
   target_pose1.position.z = vec_t.z + binMap[partLocation[_curr_part_type]].z_offset_from_part;

    // Hack
   if (target_pose1.position.y > 3.335) {
     ROS_WARN(" For some reason y position is more than 3.335");
     target_pose1.position.y = 3.1;
   }

   if (target_pose1.position.y < -3.335)
     target_pose1.position.y = -3.1;

  _manipulatorgroup.setPoseTarget(target_pose1);
   success = _manipulatorgroup.plan(my_plan);

   counter = 10;
   while (!success && counter > 0) {
       ROS_WARN("Plan not found for target pose drop location");
      success = _manipulatorgroup.plan(my_plan);
      --counter;
   }

  _manipulatorgroup.move();
   sleep(1.0);

   gripper_srv.request.enable = true;
   gripper_client.call(gripper_srv);
   sleep(2.0);
   
    // Lift the arm a little up
    ros::spinOnce();
    target_pose1.position.z = vec_t.z + binMap[partLocation[_curr_part_type]].z_offset_from_part * 4;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(0.5);
    ros::spinOnce();
  }

   if (_isPartAttached) {
   tf::Quaternion quat2(quat_s.x, quat_s.y, quat_s.z, quat_s.w);
   tf::Matrix3x3(quat2).getRPY(roll, pitch, yaw_p);
   ROS_INFO_STREAM("yaw part current: " << yaw_p);

   tf::Quaternion quat3(_home_orientation.x, _home_orientation.y, _home_orientation.z, _home_orientation.w);
   tf::Matrix3x3(quat3).getRPY(roll, pitch, yaw_e);
   ROS_INFO_STREAM("yaw end effector link: " << yaw_e);

   quat2 = tf::createQuaternionFromRPY(roll, pitch, 3.14 - yaw_pt - yaw_p); 

   target_pose1.orientation.x = quat2.x();
   target_pose1.orientation.y = quat2.y();
   target_pose1.orientation.z = quat2.z();
   target_pose1.orientation.w = quat2.w(); 

   _manipulatorgroup.setPoseTarget(target_pose1);
    success = _manipulatorgroup.plan(my_plan);
   counter = 10;
   while (!success && counter > 0) {
       ROS_WARN("Plan not found for fixing Orientation");
      success = _manipulatorgroup.plan(my_plan);
      --counter;
   }
    ROS_WARN(" No Success!");
    if (success) {
     _manipulatorgroup.move();
      sleep(1.0);
    }

    // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);
  }

  ros::spinOnce();
  sleep(1);

  bool faultyPartFound = false;
  ROS_INFO_STREAM(" Faulty Part Status AGV2 : " << _isfaultyAGV2);

  if (_isfaultyAGV1 || _isfaultyAGV2) {
    faultyPartFound = true;
   // Pick the Part again
   target_pose1.position.x = vec_t.x;
   target_pose1.position.y = vec_t.y;
   target_pose1.position.z = vec_t.z + binMap[partLocation[_curr_part_type]].z_offset_from_part;
   target_pose1.orientation = _home_orientation; 

   // Hack
   if (target_pose1.position.y > 3.335) {
     target_pose1.position.y = 3.1;
   }

   if (target_pose1.position.y < -3.335)
     target_pose1.position.y = -3.1;

   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move(); 

   gripper_srv.request.enable = true;
   gripper_client.call(gripper_srv);
   sleep(2.0);

   // Lift the arm a little up
    ros::spinOnce();
    target_pose1.position.z = vec_t.z + binMap[partLocation[_curr_part_type]].z_offset_from_part * 4;
   _manipulatorgroup.setPoseTarget(target_pose1);
   _manipulatorgroup.move();
    sleep(0.5);
    ros::spinOnce();

  if (!useAGV2)
    _manipulatorgroup.setJointValueTarget(dump_joint_values_2);
  else
    _manipulatorgroup.setJointValueTarget(dump_joint_values);

   bool success = _manipulatorgroup.plan(my_plan);
   _manipulatorgroup.move();

  // drop the part
    gripper_srv.request.enable = false;
    gripper_client.call(gripper_srv);
  }

  // Return to the Home Position
  if (!useAGV2)
      _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
  else
      _manipulatorgroup.setJointValueTarget(base_link_end_values);

  success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(1.0);
    // goHome();
  if (faultyPartFound)
    return false;

    return true;
}

bool PickAndPlace::pickAndPlace(geometry_msgs::Vector3 obj_pose, geometry_msgs::Quaternion obj_orientation, geometry_msgs::Vector3 target_pose, 
                                geometry_msgs::Quaternion target_orientation, bool useAGV2) {

  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(1.0);
   // Return to the Home Position
  if (!_conveyorPartPicked) {
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = _home_orientation;
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
    bool success = _manipulatorgroup.plan(my_plan);
    _manipulatorgroup.move();
    sleep(1.0);
  }

  ROS_INFO("Picking the Next Part");

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

  // goHome();
  std::vector<double> joint_vals = home_joint_values;
  joint_vals[0] = obj_pose.y;
  _manipulatorgroup.setJointValueTarget(joint_vals);
  bool success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
  sleep(2.0);

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

  if(!_isPartAttached)
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

 //  std::vector<double> temp_joint_values = base_link_end_values_2;
 //  temp_joint_values[2] = temp_joint_values[2] + M_PI / 3;
 //  temp_joint_values[3] = temp_joint_values[3] - M_PI / 2.5;
 //  _manipulatorgroup.setJointValueTarget(temp_joint_values);
 // _manipulatorgroup.move();
 //  sleep(1.0);
  if(!_isPartAttached)
    return false;

 // Hack
 if (target_pose.y > 3.335)
    target_pose.y = 3.3;

 if (target_pose.y < -3.335)
    target_pose.y = -3.3;

 // _manipulatorgroup.setPlanningTime(10);
 target_pose2.orientation = _home_orientation;
 target_pose2.position.x = target_pose.x;
 target_pose2.position.y = target_pose.y;
 target_pose2.position.z = target_pose.z + _z_offset_from_part;
 _manipulatorgroup.setPoseTarget(target_pose2);
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

 // Return to Base Position
  if (!useAGV2)
    _manipulatorgroup.setJointValueTarget(base_link_end_values_2);
  else
    _manipulatorgroup.setJointValueTarget(base_link_end_values);

  success = _manipulatorgroup.plan(my_plan);
  _manipulatorgroup.move();
    sleep(2.0);

  _conveyorPartPicked = false;
  return true;

}

void PickAndPlace::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr& msg) {
	_isPartAttached = msg->attached;
}

void PickAndPlace::qualSensor1(const osrf_gear::LogicalCameraImage::ConstPtr& msg) {
  _isfaultyAGV1 = (msg->models.size() > 0);
}

void PickAndPlace::qualSensor2(const osrf_gear::LogicalCameraImage::ConstPtr& msg) {
  _isfaultyAGV2 = (msg->models.size() > 0);
}

void PickAndPlace::setHome() {
  ROS_INFO("Setting Home Position");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = _home_orientation;

  // Starting Postion before activation suction. Hardcoded for now for a single piece
  target_pose1.position = _home_pt;
 _manipulatorgroup.setPoseTarget(target_pose1);
 _manipulatorgroup.move();
  sleep(2.0);
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


// Main function fo testing the PickPlace Node Standaloned
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

	PickAndPlace pickPlace(n, initialjoints, z_offset_from_part, part_location, tray_length);
	// pickPlace.pickNextPart();
  bool test = true;
  bool na = false;
  pickPlace.pickNextPartBin("gear_part", test, na);
  pickPlace.goToScanLocation();
  pickPlace.place();
	return 0;
}
 
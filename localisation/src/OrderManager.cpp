#include "localisation/OrderManager.h"

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}


OrderManager::OrderManager(ros::NodeHandle n) {
	_piston_rod_part_count = 0;
	_gear_part_count = 0;
	_curr_piston_part_count = 1;
	_curr_gear_part_count = 1;
	_once_callback_done = false;
	_actual_piston_part_count = 0;
	_actual_gear_part_count = 0;
  conveyorPartDetected = false;

  try {
      this->tf_tray_to_world.waitForTransform("/world", "logical_camera_over_agv2_kit_tray_2_frame", ros::Time(0), ros::Duration(100.0) );
  } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(10.0).sleep();
  }

  try {
    this->tf_tray_to_world.lookupTransform("/world", "logical_camera_over_agv2_kit_tray_2_frame", ros::Time(0), (this->_tray_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

  try {
      this->tf_tray_to_world.waitForTransform("/world", "logical_camera_over_agv1_kit_tray_1_frame", ros::Time(0), ros::Duration(100.0) );
  } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(1.0).sleep();
  }

  try {
    this->tf_tray_to_world.lookupTransform("/world", "/logical_camera_over_agv1_kit_tray_1_frame", ros::Time(0), (this->_tray_to_world_2));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

  // Get the location of the Logical Cameras
  try {
      this->tf_cam_bin7_to_world.waitForTransform("/world", "/logical_camera_bin7_frame", ros::Time(0), ros::Duration(30.0) );
  } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(10.0).sleep();
  }

  try {
    this->tf_cam_bin7_to_world.lookupTransform("/world", "/logical_camera_bin7_frame", ros::Time(0), (this->_cam_bin7_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

  try {
      this->tf_cam_bin6_to_world.waitForTransform("/world", "/logical_camera_bin6_frame", ros::Time(0), ros::Duration(30.0) );
  } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(10.0).sleep();
  }

  try {
    this->tf_cam_bin6_to_world.lookupTransform("/world", "/logical_camera_bin6_frame", ros::Time(0), (this->_cam_bin6_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

  try {
      this->tf_cam_bin5_to_world.waitForTransform("/world", "/logical_camera_bin5_frame", ros::Time(0), ros::Duration(30.0) );
  } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(10.0).sleep();
  }

  try {
    this->tf_cam_bin5_to_world.lookupTransform("/world", "/logical_camera_bin5_frame", ros::Time(0), (this->_cam_bin5_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }

	this->nh_ = n;
	service = nh_.advertiseService("logical_camera_server", &OrderManager::get_pose, this);
	orders_subscriber = nh_.subscribe("/ariac/orders", 10, &OrderManager::order_callback, this);
  bin7_subscriber = nh_.subscribe("/ariac/logical_camera_bin7", 10, &OrderManager::source_pose_callback_bin7, this);
  bin6_subscriber = nh_.subscribe("/ariac/logical_camera_bin6", 10, &OrderManager::source_pose_callback_bin6, this);
  logical_cam_belt_sub = nh_.subscribe("/ariac/logical_camera_over_conveyor", 1, &OrderManager::logical_camera_callback, this);
  incrementservice = nh_.advertiseService("incrementPart", &OrderManager::incrementCompletedPart, this);
}

void OrderManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    //string str1=order_msg->order_id.c_str();
    if (_once_callback_done)
    	return;

    std::stack<geometry_msgs::Pose> targetPosesGear, targetPosesPiston, targetPoses;
    for (int j = 0; j < order_msg->kits.size(); j++) {
      osrf_gear::Kit kit= order_msg->kits[j];
      int num_parts = kit.objects.size();//Number of parts to move
      //list<string>::const_iterator it;
      std::map<std::string, std::queue<geometry_msgs::Pose> > currKitPoses;
      std::vector<std::string> typeParts;

      for (int i=0;i< num_parts; i++){
        std::string type_part = kit.objects[i].type;
        ROS_INFO_STREAM(type_part);
        if (currKitPoses.find(type_part) == currKitPoses.end()) {
           std::queue<geometry_msgs::Pose> tempPoses;
           tempPoses.push(kit.objects[i].pose);
           currKitPoses.insert(make_pair(type_part, tempPoses));
           typeParts.push_back(type_part);
        } else {
          currKitPoses[type_part].push(kit.objects[i].pose);
        }

    }

     ROS_INFO_STREAM("The size of currKitPoses is : " << currKitPoses.size());
     ROS_INFO_STREAM("The size of Piston Rod Part is : " << currKitPoses["piston_rod_part"].size());
    _kits.insert(std::make_pair(j, currKitPoses));
    _kits_comp.insert(make_pair(j,typeParts));

  }
    ROS_INFO_STREAM("Size of kit: "<< _kits.size());
    // ROS_INFO_STREAM("count of gear_part:"<< _gear_part_count);
    // ROS_INFO_STREAM("Target Poses:"<< _targetPoses[0].orientation);
    _once_callback_done = true;
    _curr_kit = 0;
    _curr_kit_index = 0;
    _piston_rod_part_count = 0;
    _gear_part_count = 0;

  }

  void OrderManager::source_pose_callback_bin7(const osrf_gear::LogicalCameraImage::ConstPtr & _msg) {
      if(_msg->models.size() > 0)
        _next_pose_piston = _msg->models[0].pose;

  }

  void OrderManager::source_pose_callback_bin6(const osrf_gear::LogicalCameraImage::ConstPtr & _msg) {
      if(_msg->models.size() > 0)
        _next_pose_gear = _msg->models[0].pose;

  }

  void OrderManager::source_pose_callback_bin5(const osrf_gear::LogicalCameraImage::ConstPtr & _msg) {
      if(_msg->models.size() > 0)
        _next_pose_disk = _msg->models[0].pose;

  }

void OrderManager::logical_camera_callback(const osrf_gear::LogicalCameraImage& image_msg){
  if (!image_msg.models.empty()){
    // camera_obj = image_msg;
    //cout<<camera_obj.models[0].type;
    _obj_type_conveyor = image_msg.models[0].type;

    std::map<std::string, std::queue<geometry_msgs::Pose> > current_kit = _kits[_curr_kit];
    std::map<std::string, std::queue<geometry_msgs::Pose> >::iterator it = current_kit.begin();
    while (it != current_kit.end()) {
      if (it->first == _obj_type_conveyor) {
          conveyorPartDetected = true;
          return;
      }
      ++it;
    }
    conveyorPartDetected = false;

  }
}
bool OrderManager::isKitCompleted() {
    std::map<std::string, std::queue<geometry_msgs::Pose> > current_kit = _kits[_curr_kit];
    std::map<std::string, std::queue<geometry_msgs::Pose> >::iterator it = current_kit.begin();
    while (it != current_kit.end()) {
      if (it->second.size() > 0)
        return false;
      ++it;
    }

  return true;
}

bool OrderManager::get_pose(localisation::request_logical_pose::Request  &req, localisation::request_logical_pose::Response &res) {
  if (!_once_callback_done)
      return false;

	std::string srcFrame;
	// if (_piston_rod_part_count == 0)
	// 	return false;

	if(req.request_msg == true) {
		ROS_INFO("sending back response");

		if (isKitCompleted()) {
      _curr_kit += 1;
      if (_curr_kit < _kits.size()) {
        _curr_kit_index = 0;
      } else {
			 res.order_completed = true;
			 return true;
    }
		}

    tf::Transform targetToWorld; 
    tf::Transform sourceToWorld;

    res.noPartFound = false;

    if (conveyorPartDetected && _kits[_curr_kit][_obj_type_conveyor].size() > 0) {
      geometry_msgs::Pose pose = _kits[_curr_kit][_obj_type_conveyor].front();
      geometry_msgs::Quaternion q = pose.orientation;
      geometry_msgs::Point p = pose.position;

      tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
      tf::Vector3 vect(p.x, p.y, p.z);

      tf::Transform partToTray(quatTarget, vect);

      if ((_curr_kit) % 2 == 1)
        targetToWorld = _tray_to_world_2 * partToTray;
      else
        targetToWorld = _tray_to_world_* partToTray;
    }

		else if (_kits_comp[_curr_kit].at(_curr_kit_index).compare("piston_rod_part") == 0 && _kits[_curr_kit]["piston_rod_part"].size() > 0) {
      // geometry_msgs::Pose pose = _targetPosesPiston.top();
      geometry_msgs::Pose pose = _kits[_curr_kit]["piston_rod_part"].front();
      geometry_msgs::Quaternion q = pose.orientation;
      geometry_msgs::Point p = pose.position;

      tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
      tf::Vector3 vect(p.x, p.y, p.z);

      tf::Transform partToTray(quatTarget, vect);

      if ((_curr_kit) % 2 == 1)
        targetToWorld = _tray_to_world_2 * partToTray;
      else
        targetToWorld = _tray_to_world_* partToTray;

      // Get the Source Pose
      geometry_msgs::Quaternion q_s = _next_pose_piston.orientation;
      geometry_msgs::Point p_s = _next_pose_piston.position;

      tf::Quaternion quatSource(q_s.x, q_s.y, q_s.z, q_s.w);
      tf::Vector3 vect_s(p_s.x, p_s.y, p_s.z);

      tf::Transform partToCam(quatSource, vect_s);

      sourceToWorld = _cam_bin7_to_world_ * partToCam;

		}
    else if(_kits_comp[_curr_kit].at(_curr_kit_index).compare("gear_part") == 0 && _kits[_curr_kit]["gear_part"].size() > 0) {
      // geometry_msgs::Pose pose = _targetPosesGear.top();
      geometry_msgs::Pose pose =  _kits[_curr_kit]["gear_part"].front();
      geometry_msgs::Quaternion q = pose.orientation;
      geometry_msgs::Point p = pose.position;

      tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
      tf::Vector3 vect(p.x, p.y, p.z);

      tf::Transform partToTray(quatTarget, vect);

      if ((_curr_kit) % 2 == 1)
        targetToWorld = _tray_to_world_2 * partToTray;
      else
        targetToWorld = _tray_to_world_ * partToTray;

      // Get the Source Pose
    geometry_msgs::Quaternion q_s = _next_pose_gear.orientation;
    geometry_msgs::Point p_s = _next_pose_gear.position;

    tf::Quaternion quatSource(q_s.x, q_s.y, q_s.z, q_s.w);
    tf::Vector3 vect_s(p_s.x, p_s.y, p_s.z);

    tf::Transform partToCam(quatSource, vect_s);

    sourceToWorld = _cam_bin6_to_world_ * partToCam;
  }

  else if(_kits_comp[_curr_kit].at(_curr_kit_index).compare("disk_part") == 0 && _kits[_curr_kit]["disk_part"].size() > 0) {
      // geometry_msgs::Pose pose = _targetPosesGear.top();
      geometry_msgs::Pose pose =  _kits[_curr_kit]["disk_part"].front();
      geometry_msgs::Quaternion q = pose.orientation;
      geometry_msgs::Point p = pose.position;

      tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
      tf::Vector3 vect(p.x, p.y, p.z);

      tf::Transform partToTray(quatTarget, vect);

      if ((_curr_kit) % 2 == 1)
        targetToWorld = _tray_to_world_2 * partToTray;
      else
        targetToWorld = _tray_to_world_ * partToTray;

      // Get the Source Pose
    geometry_msgs::Quaternion q_s = _next_pose_disk.orientation;
    geometry_msgs::Point p_s = _next_pose_disk.position;

    tf::Quaternion quatSource(q_s.x, q_s.y, q_s.z, q_s.w);
    tf::Vector3 vect_s(p_s.x, p_s.y, p_s.z);

    tf::Transform partToCam(quatSource, vect_s);

    sourceToWorld = _cam_bin5_to_world_ * partToCam;
  }

  else {
    res.noPartFound = true;
    ROS_INFO_STREAM(" Current Kit Index before Modified :" << _curr_kit_index);
    ROS_INFO_STREAM(" Current Kit Size before Modified :" << _kits[_curr_kit].size());
    if (_kits[_curr_kit].size() > _curr_kit_index + 1)
      _curr_kit_index += 1;

    ROS_INFO_STREAM(" Current Kit Index Modified :" << _curr_kit_index);
  }

		geometry_msgs::Vector3 vec, vec2;
    geometry_msgs::Quaternion quat, quat2;
    tf::Quaternion q = sourceToWorld.getRotation();
    tf::Quaternion q2 = targetToWorld.getRotation();
		vec.x = sourceToWorld.getOrigin().x();
		vec.y = sourceToWorld.getOrigin().y();
		vec.z = sourceToWorld.getOrigin().z();

    vec2.x = targetToWorld.getOrigin().x();
    vec2.y = targetToWorld.getOrigin().y();
    vec2.z = targetToWorld.getOrigin().z();


    ROS_INFO_STREAM("part drop location x: "<< vec2.x);
    ROS_INFO_STREAM("part drop location y: "<< vec2.y);
    ROS_INFO_STREAM("part drop location z: "<< vec2.z);
    ROS_INFO_STREAM("Tray x: "<< _tray_to_world_.getOrigin().x());
    ROS_INFO_STREAM("Tray y: "<< _tray_to_world_.getOrigin().y());
    ROS_INFO_STREAM("Tray z: "<< _tray_to_world_.getOrigin().z());
    ROS_INFO_STREAM("Part pick location x: "<< vec.x);
    ROS_INFO_STREAM("Part pick location y: "<< vec.y);
    ROS_INFO_STREAM("Part pick location z: "<< vec.z);

    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();

    quat2.x = q2.x();
    quat2.y = q2.y();
    quat2.z = q2.z();
    quat2.w = q2.w();


		res.position = vec;
    res.orientation = quat;
    res.tgtorientation = quat2;
    res.tgtposition = vec2;
		res.order_completed = false;
    if (!conveyorPartDetected)
      res.conveyorPart = false;
    else
      res.conveyorPart = true;
	}
	else {
		ROS_INFO("No request received");
	}

	return true;
}

bool OrderManager::incrementCompletedPart(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  ROS_WARN("Came in increment service");
  if (conveyorPartDetected) {
      _kits[_curr_kit][_obj_type_conveyor].pop();
      conveyorPartDetected = false;
  }
  else {
    _kits[_curr_kit][_kits_comp[_curr_kit].at(_curr_kit_index)].pop();
  }

  if (isKitCompleted())
      response.success = true;
    else
      response.success = false;

  if (_kits[_curr_kit][_kits_comp[_curr_kit].at(_curr_kit_index)].size() == 0)
      _curr_kit_index += 1;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "order_manager");
  ros::NodeHandle n;

  OrderManager orderManager(n);
  ROS_INFO("Service to provide points to pick the next part is now being provided");
  start_competition(n);
  ros::spin();

  return 0;
}



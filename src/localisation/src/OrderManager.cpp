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


  OrderManager::OrderManager(ros::NodeHandle n, double avgManipSpeed, double acceptable_delta) {
  	_piston_rod_part_count = 0;
  	_gear_part_count = 0;
  	_curr_piston_part_count = 1;
  	_curr_gear_part_count = 1;
  	_once_callback_done = false;
  	_actual_piston_part_count = 0;
  	_actual_gear_part_count = 0;
    conveyorPartDetected = false;
    beltVeloctiyDetermined = false;
    partAccounted = false;
    partAdded = false;
    this->avgManipSpeed = avgManipSpeed; //2.1 / 3.464;
    this->acceptable_delta = acceptable_delta;
    inPlaceRotConveyor = 0.2;

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
        this->tf_cam_bin5_to_world.waitForTransform("/world", "/logical_camera_bin5_frame", ros::Time(0), ros::Duration(60.0) );
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
    highPriorityClient = nh_.serviceClient<std_srvs::Empty>("high_priority_server");
  	orders_subscriber = nh_.subscribe("/ariac/orders", 10, &OrderManager::order_callback, this);
    bin7_subscriber = nh_.subscribe("/ariac/logical_camera_bin7", 10, &OrderManager::source_pose_callback_bin7, this);
    bin6_subscriber = nh_.subscribe("/ariac/logical_camera_bin6", 10, &OrderManager::source_pose_callback_bin6, this);
    bin5_subscriber = nh_.subscribe("/ariac/logical_camera_bin5", 10, &OrderManager::source_pose_callback_bin5, this);
    logical_cam_belt_sub = nh_.subscribe("/ariac/logical_camera_over_conveyor", 1, &OrderManager::logical_camera_callback, this);
    incrementservice = nh_.advertiseService("incrementPart", &OrderManager::incrementCompletedPart, this);
  }

  void OrderManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
      ROS_WARN("New Order Received!");
      if (_once_callback_done) {
        _old_kits = _kits;
        _old_kits_comp = _kits_comp;
        _old_kit_index = _curr_kit_index;
        _old_kit = _curr_kit;

       _kits.clear();
       _kits_comp.clear();

       std_srvs::Empty highPriority;
       highPriorityClient.call(highPriority);
      }

      std::stack<geometry_msgs::Pose> targetPosesGear, targetPosesPiston, targetPoses;
      for (int j = 0; j < order_msg->kits.size(); j++) {
        osrf_gear::Kit kit= order_msg->kits[j];
        int num_parts = kit.objects.size();//Number of parts to move
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
      _kits.insert(std::make_pair(j, currKitPoses));
      _kits_comp.insert(make_pair(j,typeParts));

    }
      ROS_INFO_STREAM("Size of kit: "<< _kits.size());
      // ROS_INFO_STREAM("count of gear_part:"<< _gear_part_count);
      // ROS_INFO_STREAM("Target Poses:"<< _targetPoses[0].orientation);
      _once_callback_done = true;
      _curr_kit = 0;
      _curr_kit_index = 0;
      // _piston_rod_part_count = 0;
      // _gear_part_count = 0;

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
    if (image_msg.models.empty() && partAdded) {
       partAccounted = false;
       partAdded = false;

    } else if (!image_msg.models.empty() && !partAccounted) {
      startTime = ros::Time::now().toSec();
      partAccounted = true;
      start_pose_y = image_msg.models[0].pose.position.y;
    } else if (!image_msg.models.empty() && partAccounted && abs(image_msg.models[0].pose.position.y) < 0.001 && !partAdded) {
      // camera_obj = image_msg;
      //cout<<camera_obj.models[0].type;
       ROS_WARN("Got the point near to the centre Point");
       endTime = ros::Time::now().toSec();
      std::string obj_type_conveyor = image_msg.models[0].type;
      _conveyorPartsTime[obj_type_conveyor].push_back(endTime);
      if (std::find(_conveyorPartTypes.begin(), _conveyorPartTypes.end(), obj_type_conveyor) == _conveyorPartTypes.end())
        _conveyorPartTypes.push_back(obj_type_conveyor);
      partAdded = true;
      if (!beltVeloctiyDetermined) {
        // ROS_INFO_STREAM("The start pose is : " << start_pose_y);
        // ROS_INFO_STREAM("The start pose is : " << image_msg.models[0].pose.position.y);
        // ROS_INFO_STREAM("The start Time is : " << startTime);
        // ROS_INFO_STREAM("The end Time is : " << endTime);
        belt_velocity = std::abs(start_pose_y - image_msg.models[0].pose.position.y) / (endTime - startTime);
        ROS_INFO_STREAM(" The Belt Velocity is: " << belt_velocity);
        beltVeloctiyDetermined = true;
      }

      // std::map<std::string, std::queue<geometry_msgs::Pose> > current_kit = _kits[_curr_kit];
      // std::map<std::string, std::queue<geometry_msgs::Pose> >::iterator it = current_kit.begin();
      // while (it != current_kit.end()) {
      //   if (it->first == _obj_type_conveyor) {
      //       conveyorPartDetected = true;
      //       return;
      //   }
      //   ++it;
      // }
      // conveyorPartDetected = false;

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

  	// if (_piston_rod_part_count == 0)
  	// 	return false;

  	if (req.request_msg == true) {
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
      conveyorPartDetected = false;

      if (_conveyorPartsTime.size() > 0) {
        bool feasibleConveyorPartFound = false;
        double x = 0;
        std::list<int> erase_indices;
        for (size_t j = 0; j < _conveyorPartTypes.size() && !feasibleConveyorPartFound; j++) {
          if (_kits[_curr_kit].find(_conveyorPartTypes[j]) != _kits[_curr_kit].end() && _kits[_curr_kit][_conveyorPartTypes[j]].size() == 0)
              continue;
          std::vector<double> tempTimes = _conveyorPartsTime[_conveyorPartTypes[j]];
          for (size_t i = 0; i < tempTimes.size(); i++){
            double delta_x = belt_velocity * (inPlaceRotConveyor + (ros::Time::now().toSec() - tempTimes[i]));
            ROS_INFO_STREAM("delta_x: " << delta_x);
            if (delta_x > acceptable_delta){
              erase_indices.push_back(i);
              continue;
            }
            else {
              feasibleConveyorPartFound = true;
              erase_index = i;
              // x = (4.5 - delta_x) / (1 + belt_velocity / avgManipSpeed);
              x = (delta_x) / (1 + belt_velocity / avgManipSpeed);
              x = 2 - x;
              ROS_INFO_STREAM(" Picking Location on Conveyor : " << x);
              _obj_type_conveyor = _conveyorPartTypes[j];
              break;
            }
          }
          // TODO : Remove Parts which should no longer be considered looking at erase_indices
          // std::list<int>::iterator it2 = erase_indices.begin();
          // while (it2 ! = erase_indices.end()) {
          //   _conveyorPartsTime[_obj_type_conveyor].erase(_conveyorPartsTime.begin() + erase_index)
          // }
        }

        // The logic below was used to iterate over the _conveyorPartsTime map. Because, map under the hood is always sorted
        // This is not an ideal approach

        // std::map<std::string, std::vector<double> >::iterator it = _conveyorPartsTime.begin();
        // bool feasibleConveyorPartFound = false;
        // double x = 0;
        // std::list<int> erase_indices;
        // while (it != _conveyorPartsTime.end() && !feasibleConveyorPartFound) {
        //   std::vector<double> tempTimes = it->second;
        //   for (size_t i = 0; i < tempTimes.size(); i++){
        //     double delta_x = belt_velocity * (inPlaceRotConveyor + (ros::Time::now().toSec() - tempTimes[i]));
        //     ROS_INFO_STREAM("delta_x: " << delta_x);
        //     if (delta_x > 4){
        //       erase_indices.push_back(i);
        //       continue;
        //     }
        //     else {
        //       feasibleConveyorPartFound = true;
        //       erase_index = i;
        //       // x = (4.5 - delta_x) / (1 + belt_velocity / avgManipSpeed);
        //       x = (delta_x) / (1 + belt_velocity / avgManipSpeed);
        //       x = 2 - x;
        //       ROS_INFO_STREAM(" Picking Location on Conveyor : " << x);
        //       _obj_type_conveyor = it->first;
        //       break;
        //     }
        //   }
        //   // std::list<int>::iterator it2 = erase_indices.begin();
        //   // while (it2 ! = erase_indices.end()) {
        //   //   _conveyorPartsTime[_obj_type_conveyor].erase(_conveyorPartsTime.begin() + erase_index)
        //   // }
        //   ++it;
        // }
        ROS_WARN("Conveyor Part is going to be picked");
        ROS_INFO_STREAM("Part type: " << _obj_type_conveyor);

        if (!feasibleConveyorPartFound)
          res.noPartFound = true;

        geometry_msgs::Pose pose = _kits[_curr_kit][_obj_type_conveyor].front();
        geometry_msgs::Quaternion q = pose.orientation;
        geometry_msgs::Point p = pose.position;

        tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
        tf::Vector3 vect(p.x, p.y, p.z);

        tf::Transform partToTray(quatTarget, vect);

        if ((_curr_kit) % 2 == 0)
          targetToWorld = _tray_to_world_2 * partToTray;
        else
          targetToWorld = _tray_to_world_* partToTray;


         // TODO : Add this parameter in the launch file
         double conveyor_x = 1.21;
         double conveyor_y = x;
         double conveyor_z = 0.93;

         tf::Vector3 vec(conveyor_x, conveyor_y, conveyor_z);

         sourceToWorld.setOrigin(vec);
         conveyorPartDetected = true;
      }

  		else if (_kits_comp[_curr_kit].at(_curr_kit_index).compare("piston_rod_part") == 0 && _kits[_curr_kit]["piston_rod_part"].size() > 0) {
        // geometry_msgs::Pose pose = _targetPosesPiston.top();
        geometry_msgs::Pose pose = _kits[_curr_kit]["piston_rod_part"].front();
        geometry_msgs::Quaternion q = pose.orientation;
        geometry_msgs::Point p = pose.position;

        tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
        tf::Vector3 vect(p.x, p.y, p.z);

        tf::Transform partToTray(quatTarget, vect);

        if ((_curr_kit) % 2 == 0)
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

        if ((_curr_kit) % 2 == 0)
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

        if ((_curr_kit) % 2 == 0)
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
      // ROS_INFO_STREAM(" Current Kit Size before Modified : " << _kits[_curr_kit].size());
      if (_kits[_curr_kit].size() > _curr_kit_index + 1)
        _curr_kit_index += 1;
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
    } else {
  	   ROS_INFO("No request received");
    }

  	return true;
  }

  bool OrderManager::incrementCompletedPart(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    ROS_WARN("Came in increment service");
    if (conveyorPartDetected) {
        _kits[_curr_kit][_obj_type_conveyor].pop();
        conveyorPartDetected = false;
        _conveyorPartsTime[_obj_type_conveyor].erase(_conveyorPartsTime[_obj_type_conveyor].begin() + erase_index);
    }
    else {
      _kits[_curr_kit][_kits_comp[_curr_kit].at(_curr_kit_index)].pop();
    }

    if (isKitCompleted()) {
        response.success = true;
        if (_old_kits.size() > 0) {
            _kits = _old_kits;
            _kits_comp = _old_kits_comp;
           _curr_kit_index = _old_kit_index;
           _curr_kit = _old_kit;
      }

    } else {
        response.success = false;
      }

    if (_kits[_curr_kit][_kits_comp[_curr_kit].at(_curr_kit_index)].size() == 0)
        _curr_kit_index += 1;

    return true;
  }


  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "order_manager");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle("~");

    double avgManipSpeed, acceptable_delta;
    private_node_handle.getParam("manip_speed", avgManipSpeed);
    private_node_handle.getParam("acceptable_delta", acceptable_delta);

    OrderManager orderManager(n, avgManipSpeed, acceptable_delta);
    ROS_INFO("Service to provide points to pick the next part is now being provided");
    start_competition(n);
    ros::spin();

    return 0;
  }



#include "ariac_comp/OrderManager.h"

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
    conveyorPartDetected = false;
    beltVeloctiyDetermined = false;
    partAccounted = false;
    partAdded = false;
    this->avgManipSpeed = avgManipSpeed; //2.1 / 3.464;
    this->acceptable_delta = acceptable_delta;
    inPlaceRotConveyor = 0;
    _once_callback_done = false;
    serveHighPriority = false;
    first_part_done = false;


    // try {
    //     this->tf_tray_to_world.waitForTransform("/world", "logical_camera_over_agv2_kit_tray_2_frame", ros::Time(0), ros::Duration(100.0) );
    // } catch (tf::TransformException &ex) {
    //     ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
    //     ros::Duration(10.0).sleep();
    // }

    // try {
    //   this->tf_tray_to_world.lookupTransform("/world", "logical_camera_over_agv2_kit_tray_2_frame", ros::Time(0), (this->_tray_to_world_));
    // }

    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
    // }

   // try {
    //     this->tf_tray_to_world.waitForTransform("/world", "logical_camera_over_agv1_kit_tray_1_frame", ros::Time(0), ros::Duration(100.0) );
    // } catch (tf::TransformException &ex) {
    //     ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
    //     ros::Duration(1.0).sleep();
    // }

    // try {
    //   this->tf_tray_to_world.lookupTransform("/world", "/logical_camera_over_agv1_kit_tray_1_frame", ros::Time(0), (this->_tray_to_world_2));
    // }

    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
    // }

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

    try {
      this->tf_tray_to_world.waitForTransform("/world", "/agv1_load_point_frame", ros::Time(0), ros::Duration(20.0) );
      } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(1.0).sleep();
      }

   try {
      this->tf_tray_to_world.lookupTransform("/world", "/agv1_load_point_frame", ros::Time(0), (this->_tray_to_world_2));
      }

    catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
    }


    try {
	        tf_logical_to_world.waitForTransform("/world", "logical_camera_over_conveyor_frame", ros::Time(0), ros::Duration(50.0) );
	     } catch (tf::TransformException &ex) {
	        ROS_ERROR("[Order Manager]: (wait) %s", ex.what());
	        ros::Duration(50.0).sleep();
	  }


    try {
      tf_logical_to_world.lookupTransform("/world", "logical_camera_over_conveyor_frame", ros::Time(0), (_logical_to_world_ ));
    }

    catch (tf::TransformException &ex) {
      ROS_ERROR("[Order Manager]: (lookup) %s", ex.what());
    }

  	this->nh_ = n;
  	orders_subscriber = nh_.subscribe("/ariac/orders", 10, &OrderManager::order_callback, this);
  	next_part_service = nh_.advertiseService("next_pose_server", &OrderManager::get_next_pose, this);
    logical_cam_belt_sub = nh_.subscribe("/ariac/logical_camera_over_conveyor", 10, &OrderManager::logical_camera_callback, this);
    incrementservice = nh_.advertiseService("incrementPart", &OrderManager::incrementCompletedPart, this);
    next_part_pose_service = nh_.advertiseService("part_scan_pose", &OrderManager::partScanPose, this);

    // global timer
    start_t = ros::Time::now().toSec();

    // Get Part Locations
    FactoryFloor ff(nh_);
    partLocation = ff.getPartLocations();
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

       serveHighPriority = true;
       changedPriority = true;
      }

      for (int j = 0; j < order_msg->kits.size(); j++) {
        osrf_gear::Kit kit= order_msg->kits[j];
        int num_parts = kit.objects.size();
        std::map<std::string, std::queue<geometry_msgs::Pose> > currKitPoses;
        std::vector<std::string> typeParts;

        for (int i=0;i< num_parts; i++){
          std::string type_part = kit.objects[i].type;
          ROS_INFO_STREAM(type_part);
          // ROS_INFO_STREAM(" Position x is : " << kit.objects[i].pose.position.x);
          // ROS_INFO_STREAM(" Quternion w is : " << kit.objects[i].pose.orientation.w);

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
      _once_callback_done = true;
      _curr_kit = 0;
      _curr_kit_index = 0;

 }

 void OrderManager::logical_camera_callback(const osrf_gear::LogicalCameraImage& image_msg){
 	int index = 0;
 	bool partForScanning = false;
 	if (!image_msg.models.empty()) { 
 		// Part might have reached for Scanning
 		if (image_msg.models.size() == 1 && image_msg.models[0].pose.position.x < 0.5){
 			partForScanning = true;
 		}
 		else if (image_msg.models.size() == 2) {
 		if (image_msg.models[0].pose.position.x > image_msg.models[1].pose.position.x) {
 			index = 1;
 		}
 			partForScanning = true;
        }

        if (partForScanning) {
        	tf::Vector3 vect(image_msg.models[index].pose.position.x, image_msg.models[index].pose.position.y, image_msg.models[index].pose.position.z);
        	tf::Quaternion quat(image_msg.models[index].pose.orientation.x, image_msg.models[index].pose.orientation.y, image_msg.models[index].pose.orientation.z,image_msg.models[index].pose.orientation.w);

        	tf::Transform part_to_logical(quat, vect);
		    _part_scan_pose = _logical_to_world_ * part_to_logical;
	  		
        }
 		
 	}
 	if (partForScanning && image_msg.models.size() == 1)
 		return;

 	if (image_msg.models.size() == 2) {
 		if (index = 0)
 			index = 1;
 		else
 			index = 0;
 	}

    if (image_msg.models.empty() && partAdded) {
       partAccounted = false;
       partAdded = false;
    } else if (!image_msg.models.empty() && !partAccounted) {
      startTime = ros::Time::now().toSec();
      partAccounted = true;
      start_pose_y = image_msg.models[index].pose.position.y;
    } else if (!image_msg.models.empty() && partAccounted && abs(image_msg.models[index].pose.position.y) < 0.001 && !partAdded) {
      endTime = ros::Time::now().toSec();
      std::string obj_type_conveyor = image_msg.models[index].type;
      _conveyorPartsTime[obj_type_conveyor].push_back(endTime);
      if (std::find(_conveyorPartTypes.begin(), _conveyorPartTypes.end(), obj_type_conveyor) == _conveyorPartTypes.end())
        _conveyorPartTypes.push_back(obj_type_conveyor);

      partAdded = true;

      if (!beltVeloctiyDetermined) {
        belt_velocity = std::abs(start_pose_y - image_msg.models[index].pose.position.y) / (endTime - startTime);
        ROS_INFO_STREAM(" The Belt Velocity is: " << belt_velocity);
        beltVeloctiyDetermined = true;
      }
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

 bool OrderManager::partScanPose(ariac_comp::request_part_scan_pose::Request& req, ariac_comp::request_part_scan_pose::Response& res) {
	res.pose.translation.x = _part_scan_pose.getOrigin().getX();
	res.pose.translation.y = _part_scan_pose.getOrigin().getY();
	res.pose.translation.z = _part_scan_pose.getOrigin().getZ();

	res.pose.rotation.x = _part_scan_pose.getRotation().x();
	res.pose.rotation.y = _part_scan_pose.getRotation().y();
	res.pose.rotation.z = _part_scan_pose.getRotation().z();
	res.pose.rotation.w = _part_scan_pose.getRotation().w();
	return true;
 }

 void OrderManager::getTargetPose(std::string partType, tf::Transform& targetToWorld) {
	geometry_msgs::Pose pose = _kits[_curr_kit][partType].front();
    geometry_msgs::Quaternion q = pose.orientation;
    geometry_msgs::Point p = pose.position;

    tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
    tf::Vector3 vect(p.x, p.y, p.z);

    tf::Transform partToTray(quatTarget, vect);

    if ((_curr_kit) % 2 == 0 && !serveHighPriority)
      targetToWorld = _tray_to_world_ * partToTray;
    else
      targetToWorld = _tray_to_world_2 * partToTray;

 }

 bool OrderManager::get_next_pose(ariac_comp::request_next_pose::Request  &req, ariac_comp::request_next_pose::Response &res) {
    // ROS_WARN("Getting the Next Target Pose");
    if (!_once_callback_done || (ros::Time::now().toSec() - start_t) < 5)
        return false;

    if (changedPriority)
		changedPriority = false;

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

      bool feasibleConveyorPartFound = false;
      if (_conveyorPartsTime.size() > 0 && partLocation.find(_kits_comp[_curr_kit].at(_curr_kit_index)) == partLocation.end()) {
        double x = 0;
        std::list<int> erase_indices;
        for (size_t j = 0; j < _conveyorPartTypes.size() && !feasibleConveyorPartFound; j++) {
          if (_kits[_curr_kit].find(_conveyorPartTypes[j]) == _kits[_curr_kit].end()) //  && _kits[_curr_kit][_conveyorPartTypes[j]].size() == 0)
              continue;

          if (partLocation.find(_conveyorPartTypes[j]) != partLocation.end())
              continue;

          std::vector<double> tempTimes = _conveyorPartsTime[_conveyorPartTypes[j]];
          for (size_t i = 0; i < tempTimes.size(); i++){
            double dist_travelled_part, delta_x;
            if ((_curr_kit) % 2 == 1 || serveHighPriority) 
            	dist_travelled_part	= belt_velocity * (inPlaceRotConveyor + (ros::Time::now().toSec() - tempTimes[i]));
            else 
            	dist_travelled_part = 4.2 - belt_velocity * (inPlaceRotConveyor + (ros::Time::now().toSec() - tempTimes[i]));

            if (std::abs(dist_travelled_part) > acceptable_delta){
              erase_indices.push_back(i);
              continue;
            }
            else {
              feasibleConveyorPartFound = true;
              erase_index = i;
              if ((_curr_kit) % 2 == 1 || serveHighPriority) {
                delta_x = (dist_travelled_part) / (avgManipSpeed / belt_velocity - 1);
                x = dist_travelled_part + delta_x;
              }
              else {
                delta_x = (dist_travelled_part) / (avgManipSpeed / belt_velocity + 1);
                x = dist_travelled_part - delta_x;
              }

              _obj_type_conveyor = _conveyorPartTypes[j];
              res.partType = _obj_type_conveyor;
              break;
            }
          }
          // TODO : Remove Parts which should no longer be considered looking at erase_indices
          // std::list<int>::iterator it2 = erase_indices.begin();
          // while (it2 ! = erase_indices.end()) {
          //   _conveyorPartsTime[_obj_type_conveyor].erase(_conveyorPartsTime.begin() + erase_index)
          // }
        }

        if (feasibleConveyorPartFound) {
	        ROS_WARN("Conveyor Part is going to be picked");
	        ROS_INFO_STREAM("Part type: " << _obj_type_conveyor);

	        getTargetPose(_obj_type_conveyor, targetToWorld);

	         // TODO : Add this parameter in the launch file
	         double conveyor_x = 1.21;
	         double conveyor_y = x;
	         double conveyor_z = 0.93;

	         tf::Vector3 vec(conveyor_x, conveyor_y, conveyor_z);

	         sourceToWorld.setOrigin(vec);
	         conveyorPartDetected = true;
        }
      }

      if (!feasibleConveyorPartFound && partLocation.find(_kits_comp[_curr_kit].at(_curr_kit_index)) != partLocation.end()) {

	      if (_kits_comp[_curr_kit].at(_curr_kit_index).compare("piston_rod_part") == 0 && _kits[_curr_kit]["piston_rod_part"].size() > 0) {
	  	 	getTargetPose("piston_rod_part", targetToWorld);
		   _last_part_type = "piston_rod_part";
		   res.partType = "piston_rod_part";

	  	   }

	     else if(_kits_comp[_curr_kit].at(_curr_kit_index).compare("gear_part") == 0 && _kits[_curr_kit]["gear_part"].size() > 0) {
	     	 getTargetPose("gear_part", targetToWorld);
	      	_last_part_type = "gear_part";
	      	res.partType = "gear_part";

	     }

	    else if(_kits_comp[_curr_kit].at(_curr_kit_index).compare("disk_part") == 0 && _kits[_curr_kit]["disk_part"].size() > 0) {
	    	 getTargetPose("disk_part", targetToWorld);
	      	_last_part_type = "disk_part";
	      	res.partType = "disk_part";
	      	
	    }

	    else if(_kits_comp[_curr_kit].at(_curr_kit_index).compare("gasket_part") == 0 && _kits[_curr_kit]["gasket_part"].size() > 0) {
	    	 getTargetPose("gasket_part", targetToWorld);
	      	_last_part_type = "gasket_part";
	      	res.partType = "gasket_part";
	    }

      else if(_kits_comp[_curr_kit].at(_curr_kit_index).compare("pulley_part") == 0 && _kits[_curr_kit]["pulley_part"].size() > 0) {
         getTargetPose("pulley_part", targetToWorld);
          _last_part_type = "pulley_part";
          res.partType = "pulley_part";
      }

	    else {
	      res.noPartFound = true;
	      // ROS_INFO_STREAM(" Current Kit Size before Modified : " << _kits[_curr_kit].size());
	      if (_kits[_curr_kit].size() > _curr_kit_index + 1)
	        _curr_kit_index += 1;
	    }
      }

      if (!feasibleConveyorPartFound && partLocation.find(_kits_comp[_curr_kit].at(_curr_kit_index)) == partLocation.end() && first_part_done) { //_curr_kit != 0) {
         // Part Not Attainable/Achievable
        _kits[_curr_kit][_kits_comp[_curr_kit].at(_curr_kit_index)].pop();
        _curr_kit_index += 1;
        return false;
      }

  	 geometry_msgs::Vector3 vec, vec2;
     geometry_msgs::Quaternion quat, quat2;
     tf::Quaternion q = sourceToWorld.getRotation().normalize();
     tf::Quaternion q2 = targetToWorld.getRotation().normalize();

     vec.x = sourceToWorld.getOrigin().x();
  	 vec.y = sourceToWorld.getOrigin().y();
  	 vec.z = sourceToWorld.getOrigin().z();

     vec2.x = targetToWorld.getOrigin().x();
     vec2.y = targetToWorld.getOrigin().y();
     vec2.z = targetToWorld.getOrigin().z();

     // Hack
    if ((_curr_kit) % 2 == 1 || serveHighPriority)
      vec2.y = vec2.y - 0.15;
    else
      vec2.y = vec2.y + 0.15;

      // ROS_INFO_STREAM("part drop location x: "<< vec2.x);
      // ROS_INFO_STREAM("part drop location y: "<< vec2.y);
      // ROS_INFO_STREAM("part drop location z: "<< vec2.z);

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

      if (serveHighPriority)
      	res.highPriority = true;
      else
      	res.highPriority = false;

  	return true;
  }

  bool OrderManager::incrementCompletedPart(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    ROS_WARN("Came in increment service");
    first_part_done = true;
    if (conveyorPartDetected) {
        _kits[_curr_kit][_obj_type_conveyor].pop();
        conveyorPartDetected = false;
        _conveyorPartsTime[_obj_type_conveyor].erase(_conveyorPartsTime[_obj_type_conveyor].begin() + erase_index);
    }
    else {
      if (changedPriority) {
          _old_kits[_old_kit][_old_kits_comp[_old_kit].at(_old_kit_index)].pop();
          changedPriority = false;
        }
        else {
          _kits[_curr_kit][_kits_comp[_curr_kit].at(_curr_kit_index)].pop();
        }
    }

    if (isKitCompleted()) { 
        response.success = true;
    	if (_old_kits.size() > 0) {
           _kits = _old_kits;
           _kits_comp = _old_kits_comp;
           _curr_kit_index = _old_kit_index;
           _curr_kit = _old_kit;
           serveHighPriority = false;
           _old_kits.clear();
           _old_kits_comp.clear();
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
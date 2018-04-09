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

  try {
      this->tf_tray_to_world.waitForTransform("/world", "/agv2_load_point_frame", ros::Time(0), ros::Duration(20.0) );
  } catch (tf::TransformException &ex) {
      ROS_ERROR("[pick_and_place]: (wait) %s", ex.what());
      ros::Duration(10.0).sleep();
  }

  try {
    this->tf_tray_to_world.lookupTransform("/world", "/agv2_load_point_frame", ros::Time(0), (this->_tray_to_world_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[pick_and_place]: (lookup) %s", ex.what());
  }


	this->nh_ = n;
	service = nh_.advertiseService("logical_camera_server", &OrderManager::get_pose, this);
	orders_subscriber = nh_.subscribe("/ariac/orders", 10, &OrderManager::order_callback, this);
  incrementservice = nh_.advertiseService("incrementPart", &OrderManager::incrementCompletedPart, this);
}

void OrderManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    //string str1=order_msg->order_id.c_str();
    if (_once_callback_done)
    	return;

    osrf_gear::Kit kit1= order_msg->kits[0];
    int num_parts = kit1.objects.size();//Number of parts to move
    ROS_INFO_STREAM("Number of parts: "<< num_parts);
    //list<string>::const_iterator it;
    for (int i=0;i< num_parts; i++){
    std::string type_kit;
    type_kit=kit1.objects[i].type;
    ROS_INFO_STREAM(type_kit);
    if (type_kit=="gear_part") {
      _gear_part_count++;// countof prp
      _targetPosesGear.push(kit1.objects[i].pose);
    }
    if (type_kit=="piston_rod_part") {
      _piston_rod_part_count++;//count of gear_part
      _targetPosesPiston.push(kit1.objects[i].pose);
    }

    }
    ROS_INFO_STREAM("count of piston_rod_part: "<< _piston_rod_part_count);
    ROS_INFO_STREAM("count of gear_part:"<< _gear_part_count);
    // ROS_INFO_STREAM("Target Poses:"<< _targetPoses[0].orientation);
    _once_callback_done = true;

  }

bool OrderManager::get_pose(localisation::request_logical_pose::Request  &req, localisation::request_logical_pose::Response &res) {
	std::string srcFrame;
	if (_piston_rod_part_count == 0)
		return false;

	if(req.request_msg == true) {
		ROS_INFO("sending back response");
		ROS_INFO_STREAM("Piston Completed: " << _actual_piston_part_count);
		// geometry_msgs::TransformStamped obj_pose;

		if (_actual_piston_part_count == _piston_rod_part_count && _actual_gear_part_count == _gear_part_count) {
			res.order_completed = true;
			return true;
		}

    tf::Transform targetToWorld; 
		if (_actual_piston_part_count < _piston_rod_part_count) {
      geometry_msgs::Pose pose = _targetPosesPiston.top();
      geometry_msgs::Quaternion q = pose.orientation;
      geometry_msgs::Point p = pose.position;

      tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
      tf::Vector3 vect(p.x, p.y, p.z);

      tf::Transform partToTray(quatTarget, vect);

      targetToWorld = _tray_to_world_ * partToTray;

			std::string part_count_string;
			std::stringstream ss;
			ss <<_curr_piston_part_count;
			ss >> part_count_string;
        	srcFrame = std::string("/logical_camera_bin7_piston_rod_part_") + part_count_string + "_frame";
        	_curr_piston_part_count++;
        	ROS_INFO_STREAM(srcFrame);
		}
    else if(_actual_gear_part_count < _gear_part_count) {
      geometry_msgs::Pose pose = _targetPosesGear.top();
      geometry_msgs::Quaternion q = pose.orientation;
      geometry_msgs::Point p = pose.position;

      tf::Quaternion quatTarget(q.x, q.y, q.z, q.w);
      tf::Vector3 vect(p.x, p.y, p.z);

      tf::Transform partToTray(quatTarget, vect);

      targetToWorld = _tray_to_world_ * partToTray;
      std::string part_count_string;
			std::stringstream ss;
			ss <<_curr_gear_part_count;
			ss >> part_count_string;
        	srcFrame = std::string("/logical_camera_bin6_gear_part_") + part_count_string + "_frame";
        	_curr_gear_part_count++;
        }

        try {
    	this->tf_logical_to_world.waitForTransform("/world", srcFrame, ros::Time(0), ros::Duration(10.0) );
 		 } catch (tf::TransformException &ex) {
    	ROS_ERROR("[localisation]: (wait) %s", ex.what());
    	ros::Duration(10.0).sleep();
  		}

  		try {
    		this->tf_logical_to_world.lookupTransform("/world", srcFrame, ros::Time(0), (this->_logical_to_world_));
  		}

  		catch (tf::TransformException &ex) {
    		ROS_ERROR("[localisation]: (lookup) %s", ex.what());
  		}

		// res.position = obj_pose.transform.translation;
		geometry_msgs::Vector3 vec, vec2;
    geometry_msgs::Quaternion quat, quat2;
    tf::Quaternion q = _logical_to_world_.getRotation();
    tf::Quaternion q2 = targetToWorld.getRotation();
		vec.x = _logical_to_world_.getOrigin().x();
		vec.y = _logical_to_world_.getOrigin().y();
		vec.z = _logical_to_world_.getOrigin().z();

    vec2.x = targetToWorld.getOrigin().x();
    vec2.y = targetToWorld.getOrigin().y();
    vec2.z = targetToWorld.getOrigin().z();


    ROS_INFO_STREAM("part drop location x: "<< vec2.x);
    ROS_INFO_STREAM("part drop location y: "<< vec2.y);
    ROS_INFO_STREAM("part drop location z: "<< vec2.z);
    ROS_INFO_STREAM("Tray location x: "<< _tray_to_world_.getOrigin().getX());
    ROS_INFO_STREAM("Tray location y: "<< _tray_to_world_.getOrigin().getY());
    ROS_INFO_STREAM("Tray location z: "<< _tray_to_world_.getOrigin().getZ());

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
	}
	else {
		ROS_INFO("No request received");
	}

	return true;
}

bool OrderManager::incrementCompletedPart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  ROS_WARN("Came in increment service");
  if (_actual_piston_part_count < _piston_rod_part_count) {
    _actual_piston_part_count++;
    _targetPosesPiston.pop();
  }
  else if (_actual_gear_part_count < _gear_part_count){
    _actual_gear_part_count++;
    _targetPosesGear.pop();
  }
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



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
    std::string type_kits[num_parts];
    type_kits[i]=kit1.objects[i].type;
    ROS_INFO_STREAM(type_kits[i]);
    if (type_kits[i]=="gear_part")_gear_part_count++;// countof prp
    if (type_kits[i]=="piston_rod_part") _piston_rod_part_count++;//count of gear_part
    }
    ROS_INFO_STREAM("count of piston_rod_part: "<< _piston_rod_part_count);
    ROS_INFO_STREAM("count of gear_part:"<< _gear_part_count);

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

		if (_actual_piston_part_count < _piston_rod_part_count) {
			std::string part_count_string;
			std::stringstream ss;
			ss <<_curr_piston_part_count;
			ss >> part_count_string;
        	srcFrame = std::string("/logical_camera_bin7_piston_rod_part_") + part_count_string + "_frame";
        	_curr_piston_part_count++;
        	ROS_INFO_STREAM(srcFrame);
		}
    else if(_actual_gear_part_count < _gear_part_count) {
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
    	ros::Duration(1.0).sleep();
  		}

  		try {
    		this->tf_logical_to_world.lookupTransform("/world", srcFrame, ros::Time(0), (this->_logical_to_world_));
  		}

  		catch (tf::TransformException &ex) {
    		ROS_ERROR("[localisation]: (lookup) %s", ex.what());
  		}

		// res.position = obj_pose.transform.translation;
		geometry_msgs::Vector3 vec;
		vec.x = _logical_to_world_.getOrigin().x();
		vec.y = _logical_to_world_.getOrigin().y();
		vec.z = _logical_to_world_.getOrigin().z();
		res.position = vec;
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
  }
  else if (_actual_gear_part_count < _gear_part_count){
    _actual_gear_part_count++;
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



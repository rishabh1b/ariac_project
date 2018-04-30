#include "ariac_comp/PickAndPlace.h"
#include "ariac_comp/request_next_pose.h"
#include "osrf_gear/AGVControl.h"
#include <std_srvs/Trigger.h>
#include <sstream>

class OrderFullfiller {
private:
	ros::ServiceClient nextPartclient;
	ros::ServiceClient incrementclient;
	bool _once_callback_done;
	tf::TransformListener tf_logical_to_world;
	tf::StampedTransform _logical_to_world_;
	ros::Subscriber orders_subscriber;
	
	ros::ServiceClient submissionclient;
	ros::ServiceServer highPriorityServer;
	ariac_comp::request_next_pose pointsrv;
	std_srvs::Trigger incPart;
    osrf_gear::AGVControl subsrv;
	int kit_num;
	bool usedAGV2;
	bool highPriorityOrderReceived;
	ros::NodeHandle n;

	// Target And Source Poses
    geometry_msgs::Vector3 obj_pose, target_pose;
    geometry_msgs::Quaternion obj_orientation;
	geometry_msgs::Quaternion target_orientation;

public :
	 OrderFullfiller(ros::NodeHandle n);

    // New Service
    // bool highPriorityService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    // 	ROS_WARN("Switching to High Priority Order");
    // 	highPriorityOrderReceived = true;
    // }


    bool manage(PickAndPlace& pickPlace);
	    
	
};
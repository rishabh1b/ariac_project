#include "ariac_comp/PickAndPlace.h"
#include "ariac_comp/request_next_pose.h"
#include "ariac_comp/request_part_scan_pose.h"
#include "osrf_gear/AGVControl.h"
#include <std_srvs/Trigger.h>
#include <sstream>

class OrderFullfiller {
private:
	ros::ServiceClient nextPartclient, incrementclient, partPoseclient;
	bool _once_callback_done, conveyorPickingPositionAttained;
	tf::TransformListener tf_logical_to_world;
	tf::StampedTransform _logical_to_world_;
	ros::Subscriber orders_subscriber;
	
	ros::ServiceClient submissionclient;
	ros::ServiceServer highPriorityServer;
	ariac_comp::request_next_pose pointsrv;
	ariac_comp::request_part_scan_pose scanPose;
	std_srvs::Trigger incPart;
    osrf_gear::AGVControl subsrv;
	int kit_num, kit_num_h;
	bool useAGV2;
	bool highPriorityOrderReceived;
	ros::NodeHandle n;

	// Target And Source Poses
    geometry_msgs::Vector3 obj_pose, target_position;
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
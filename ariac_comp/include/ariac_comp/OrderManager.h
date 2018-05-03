#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <iostream> 
#include <stdio.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Kit.h>
#include <osrf_gear/KitObject.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <string>
#include <list>
#include <stack>
#include <queue>
#include <map>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include "ariac_comp/request_next_pose.h"
#include <cmath> 

class OrderManager {
	private:
		int _curr_kit, _curr_kit_index;
		size_t erase_index;
		tf::TransformListener tf_logical_to_world, tf_tray_to_world;
		tf::StampedTransform _logical_to_world_, _ee_to_base_ ,_error_in_eelink, _tray_to_world_,_tray_to_world_2;
		tf::Vector3 _part_scan_pose, _part_scan_pose_e;
		ros::NodeHandle nh_;
		ros::Subscriber logical_cam_belt_sub, orders_subscriber;
		ros::ServiceServer next_part_service, incrementservice;

		std::map<int, std::map<std::string, std::queue<geometry_msgs::Pose> > > _kits; 
		std::map<int, std::vector<std::string> > _kits_comp;
		std::map<std::string, std::vector<double> > _conveyorPartsTime;
		std::vector<std::string> _conveyorPartTypes;

		bool isKitCompleted();

		std::string _obj_type_conveyor, _last_part_type;
		bool conveyorPartDetected, beltVeloctiyDetermined, partAccounted, partAdded, _once_callback_done;
		double belt_velocity, startTime, endTime, start_pose_y, avgManipSpeed, inPlaceRotConveyor, acceptable_delta;

	public:
		OrderManager(ros::NodeHandle n, double avgManipSpeed = 0.606, double acceptable_delta = 2);
		void order_callback(const osrf_gear::Order::ConstPtr & order_msg);
		void logical_camera_callback(const osrf_gear::LogicalCameraImage& image_msg);
		bool get_next_pose(ariac_comp::request_next_pose::Request  &req, ariac_comp::request_next_pose::Response &res);
		bool incrementCompletedPart(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
};
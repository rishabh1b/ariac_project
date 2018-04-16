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
#include "localisation/request_logical_pose.h"
#include <geometry_msgs/Pose.h>

class OrderManager {
	private:
		int _piston_rod_part_count, _gear_part_count, _curr_kit, _curr_kit_index;
		int _curr_piston_part_count, _curr_gear_part_count, _actual_piston_part_count, _actual_gear_part_count;
		bool _once_callback_done;
		tf::TransformListener tf_logical_to_world;
		tf::StampedTransform _logical_to_world_;
		ros::NodeHandle nh_;
		ros::ServiceServer service;
		ros::ServiceServer incrementservice;
		ros::Subscriber orders_subscriber;
		ros::Subscriber bin7_subscriber;
		ros::Subscriber bin6_subscriber;
		ros::Subscriber logical_cam_belt_sub;
		tf::TransformListener tf_tray_to_world;
		tf::TransformListener tf_cam_bin7_to_world;
		tf::TransformListener tf_cam_bin6_to_world, tf_cam_bin5_to_world;
		tf::StampedTransform _tray_to_world_,_tray_to_world_2;
		tf::StampedTransform _cam_bin7_to_world_;
		tf::StampedTransform _cam_bin6_to_world_, _cam_bin5_to_world_;

	    std::stack<geometry_msgs::Pose> targetPoses; //_targetPosesPiston, _targetPosesGear;

		geometry_msgs::Pose _next_pose_piston, _next_pose_gear, _next_pose_disk;

		std::map<int, std::map<std::string, std::queue<geometry_msgs::Pose> > > _kits; 
		std::map<int, std::vector<std::string> > _kits_comp;
		bool isKitCompleted();

		std::string _obj_type_conveyor;
		bool conveyorPartDetected;

	public:
		OrderManager(ros::NodeHandle n);
		void  order_callback(const osrf_gear::Order::ConstPtr & order_msg);
		void  source_pose_callback_bin7(const osrf_gear::LogicalCameraImage::ConstPtr & _msg);
		void  source_pose_callback_bin6(const osrf_gear::LogicalCameraImage::ConstPtr & _msg);
		void  source_pose_callback_bin5(const osrf_gear::LogicalCameraImage::ConstPtr & _msg);
		void  logical_camera_callback(const osrf_gear::LogicalCameraImage& image_msg);
		bool  get_pose(localisation::request_logical_pose::Request  &req, localisation::request_logical_pose::Response &res);
		// bool incrementCompletedPart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool incrementCompletedPart(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
};
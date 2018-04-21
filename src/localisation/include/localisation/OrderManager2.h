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
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include "localisation/request_logical_pose.h"

class OrderManager {
	private:
		int _piston_rod_part_count, _gear_part_count;
		int _curr_piston_part_count, _curr_gear_part_count, _actual_piston_part_count, _actual_gear_part_count;
		bool _once_callback_done;
		tf::TransformListener tf_logical_to_world;
		tf::StampedTransform _logical_to_world_;
		ros::NodeHandle nh_;
		ros::ServiceServer service;
		ros::ServiceServer incrementservice;
		ros::Subscriber orders_subscriber;
	public:
		OrderManager(ros::NodeHandle n);
		void  order_callback(const osrf_gear::Order::ConstPtr & order_msg);
		bool  get_pose(localisation::request_logical_pose::Request  &req, localisation::request_logical_pose::Response &res);
		bool incrementCompletedPart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};
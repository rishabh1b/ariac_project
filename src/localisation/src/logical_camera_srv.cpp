//============================================================================
// Name        : abc.cpp
// Author      : Anurag Bansal
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <algorithm>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include "localisation/request_logical_pose.h"

using namespace std;

ros::ServiceServer service; //service definition
ros::Subscriber sub; //subscriber definition
osrf_gear::LogicalCameraImage camera_obj; //logical camera image data
geometry_msgs::Vector3 obj_pose;

/*Used to get transform from bin frame to world frame for robot*/
geometry_msgs::Vector3 get_pose_transform(geometry_msgs::Vector3 tray_pose, /*object_type*/) {
	tf::TransformListener listener;
  	tf::StampedTransform transform;
  	geometry_msgs::PointStamped base_point;//for tranform operation
  	geometry_msgs::Vector3 robot_base_point; //to store only the x,y,z
  	geometry_msgs::PointStamped logical_camera_point;
  	logical_camera_point.header.frame_id = "/logical_camera";
  	logical_camera_point.header.stamp = ros::Time();
  	logical_camera_point.point.x = tray_pose.x;
  	logical_camera_point.point.y = tray_pose.y;
  	logical_camera_point.point.z = tray_pose.z;
   	try {
    	/*if (object_type == x)*/
    	listener.waitForTransform("/logical_camera_bin6_frame", "/world",
    	ros::Time(0), ros::Duration(10.0) );
    	/*else*/
    	/*listener.waitForTransform("/logical_camera_bin7_frame", "/world",
    	ros::Time(0), ros::Duration(10.0) );*/			
    	listener.transformPoint("/world",logical_camera_point,base_point);
  	}
  	catch (tf::TransformException &ex) {
    	ROS_ERROR("%s",ex.what());
    	ros::Duration(1.0).sleep();
  	}
  	robot_base_point.x = base_point.point.x;
  	robot_base_point.y = base_point.point.y;
  	robot_base_point.z = base_point.point.z;
  	cout<<robot_base_point.x<<endl;
  	cout<<robot_base_point.y<<endl;
  	cout<<robot_base_point.z<<endl;
  	return robot_base_point;
}

/*Server for sending tanformed poses*/
bool get_pose(localisation::request_logical_pose::Request  &req, localisation::request_logical_pose::Response &res){
	if(req.request_msg == true) {
		ROS_INFO("sending back response");
		obj_pose = get_pose_transform(obj_pose,/*object_type*/);
		res.position = obj_pose;
	}
	else {
		ROS_INFO("No request received");
	}
	ros::spinOnce();
	ros::Duration(0.2).sleep();
	return true;
}

/*Logical Camera Callback function*/
void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg){

	camera_obj = *image_msg;
	obj_pose.x = camera_obj.pose.position.x;
	obj_pose.y = camera_obj.pose.position.y;
	obj_pose.z = camera_obj.pose.position.z;
	//cout<<obj_pose.x<<endl;
	//cout<<obj_pose.y<<endl;
	//cout<<obj_pose.z<<endl;
	ROS_INFO_STREAM_THROTTLE(10,"Logical camera: '" << image_msg->models.size() << "' objects.");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "logical_camera_node");
	ros::NodeHandle nh;
	//add condition based on the part to be provided by Indu
	/*if(part == x)*/ 
		sub = nh.subscribe("/ariac/logical_camera_bin6", 1, logical_camera_callback);
	/*else*/
		//sub = nh.subscribe("/ariac/logical_camera_bin7", 1, logical_camera_callback);
	service = nh.advertiseService("logical_camera_server", get_pose);
	ros::spin();
}
#include <algorithm>
#include <vector>
#include <cmath> 
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

//ros::ServiceServer service; //service definition
ros::Subscriber sub; //subscriber definition
osrf_gear::LogicalCameraImage camera_obj; //logical camera image data
geometry_msgs::Vector3 obj_pose_conveyor;
float start_time;
float stop_time;
float conveyor_velocity;
float y_start, y_stop;
bool part_entered = false;
bool part_exited = false;

/*void get_velocity(){
	if (!camera_obj.models.empty() && !part_exited){
		//ROS_ERROR("Part entered Logical camera");
      	obj_pose_conveyor.y = camera_obj.models[0].pose.position.y;
      	part_entered = true;
      	part_exited = false;
      	start_time = ros::Time::now().toSec();
      	y_start = obj_pose_conveyor.y;
      	//ROS_ERROR("starttime, %f",start_time);
      }
      //ros::spinOnce();
      
      if(camera_obj.models.empty() && part_entered){
      	ROS_WARN("Part exited Logical camera");
      	stop_time = ros::Time::now().toSec();
      	y_stop = obj_pose_conveyor.y;
      	part_exited = true;
      	part_entered = false;
      	ROS_WARN("stoptime, %f",stop_time);
      }
      if(part_exited){
      	conveyor_velocity = abs(y_stop - y_start)/(start_time - stop_time);
      	ROS_WARN("y_stop %f, y_start %f",y_stop,y_start);
      	ROS_ERROR("velocity = %f",conveyor_velocity);
  	}
}*/

float get_one_sec_velocity(){
		y_start = base_point.point.y;
		ros::Duration(1).sleep();
		ros::spinOnce();
		y_stop = base_point.point.y;
		conveyor_velocity = y_start - y_stop;
		//ROS_ERROR("velocity = %f",conveyor_velocity);
		if(conveyor_velocity > 0){
			float time_to_zero = y_stop/conveyor_velocity;
			return time_to_zero;
		}
		else
			return 0;	
}

float get_time_to_zero(){

}


void logical_camera_callback(const osrf_gear::LogicalCameraImage& image_msg){
	if (!image_msg.models.empty()){
		camera_obj = image_msg;
		//cout<<camera_obj.models[0].type;
		obj_pose_conveyor.x = camera_obj.models[0].pose.position.x;
		obj_pose_conveyor.y = camera_obj.models[0].pose.position.y;
		obj_pose_conveyor.z = camera_obj.models[0].pose.position.z;
		tf::TransformListener listener;
  		tf::StampedTransform transform;
  		geometry_msgs::PointStamped base_point;//for tranform operation
  		geometry_msgs::PointStamped logical_camera_point;
  		logical_camera_point.header.frame_id = "/logical_camera_over_conveyor_frame";
  		logical_camera_point.header.stamp = ros::Time();
  		logical_camera_point.point.x = obj_pose_conveyor.x;
  		logical_camera_point.point.y = obj_pose_conveyor.y;
  		logical_camera_point.point.z = obj_pose_conveyor.z;
  		try {
    	/*if (object_type == x)*/
    	listener.waitForTransform("/logical_camera_over_conveyor_frame", "/world",
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
  	ROS_INFO("x = %f y = %f z = %f", base_point.point.x,base_point.point.y,base_point.point.z);
  	//cout<<base_point.point.y<<endl;
  	//cout<<base_point.point.z<<endl;
		//cout<<obj_pose_conveyor.y<<endl;
	}
	//else
		//camera_obj = osrf_gear::LogicalCameraImage();
}


/*void logical_camera_callback(const osrf_gear::LogicalCameraImage& image_msg){
	if (!image_msg.models.empty()){
		camera_obj = image_msg;
		//cout<<camera_obj.models[0].type;
		obj_pose_conveyor.y = camera_obj.models[0].pose.position.y;
		//cout<<obj_pose_conveyor.y<<endl;
	}
	//else
		//camera_obj = osrf_gear::LogicalCameraImage();
}*/

int main(int argc, char **argv){
	ros::init(argc, argv, "conveyor_velocity_node");
	ros::NodeHandle nh;
	//add condition based on the part to be provided by Indu
	/*if(part == x)*/ 
	sub = nh.subscribe("/ariac/logical_camera_over_conveyor", 1, logical_camera_callback);
	while(ros::ok()){
        ros::spinOnce();
        //cout<<camera_obj.models[0];
		float time_to_zero = get_one_sec_velocity();
		cout<<time_to_zero<<endl;
	}
	/*else*/
		//sub = nh.subscribe("/ariac/logical_camera_bin7", 1, logical_camera_callback);
	//service = nh.advertiseService("logical_camera_server", get_pose);
	ros::spin();
}
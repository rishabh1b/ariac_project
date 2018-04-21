#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <geometry_msgs/Vector3.h>
#include <osrf_gear/VacuumGripperState.h>

class PickAndPlace {
private:
	ros::NodeHandle nh_;
	geometry_msgs::Pose _homePose;
	geometry_msgs::Quaternion _home_orientation, _tray_orientation;
	geometry_msgs::Vector3 _home_position;
	geometry_msgs::Point _home_pt;
	osrf_gear::VacuumGripperControl gripper_srv;
	ros::ServiceClient gripper_client;
	ros::Subscriber gripperStateSubscriber;
	double _z_offset_from_part;

	moveit::planning_interface::MoveGroup _manipulatorgroup;
	tf::StampedTransform _tray_to_world_, _tray_to_world_2;
	tf::Quaternion _home_quat;

	tf::TransformListener tf_tray_to_world;

	moveit::planning_interface::MoveGroup::Plan my_plan;

	float _tray_location_x, _tray_location_y, _tray_location_z, _tray_length, _tray_1_x, _tray_1_y, _tray_1_z;

	double test_x, test_y, test_z, conveyor_x, conveyor_y, conveyor_z;

	std::vector<double> home_joint_values, base_link_end_values, base_link_end_values_2, return_home_joint_values, conveyor_joint_values;
	int index;

	bool _isPartAttached, _nowExecuting, _conveyorPartPicked;
	
	void initialSetup();
	void goHome();
	void goHome2();
	void setHome(); 
	float getRandomValue();

public:
	PickAndPlace(ros::NodeHandle n, double* initialPositions, double _z_offset_from_part, double* part_location, float tray_length = 0.2);
	void performPickAndPlace();
	void pickNextPart();
	bool pickPlaceNextPartConveyor(geometry_msgs::Vector3 obj_pose, geometry_msgs::Vector3 target_pose, 
                                            geometry_msgs::Quaternion target_orientation, bool useAGV2);
	bool place();
	bool place(geometry_msgs::Vector3 vec, geometry_msgs::Quaternion quat);
	bool pickNextPart(geometry_msgs::Vector3 obj_pose);
	bool pickNextPart(geometry_msgs::Vector3 obj_pose, geometry_msgs::Quaternion orientation);
	bool pickAndPlace(geometry_msgs::Vector3 obj_pose, geometry_msgs::Quaternion obj_orientation, geometry_msgs::Vector3 target_pose, 
                      geometry_msgs::Quaternion target_orientation, bool useAGV2 = true);
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr& msg);
};
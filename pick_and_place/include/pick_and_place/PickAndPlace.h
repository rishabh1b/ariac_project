#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <geometry_msgs/Vector3.h>

class PickAndPlace {
private:
	ros::NodeHandle nh_;
	geometry_msgs::Pose _homePose;
	geometry_msgs::Quaternion _home_orientation;
	osrf_gear::VacuumGripperControl gripper_srv;
	ros::ServiceClient gripper_client;
	double _z_offset_from_part;

	moveit::planning_interface::MoveGroup _manipulatorgroup;
	tf::StampedTransform _tray_to_world_;

	tf::TransformListener tf_tray_to_world;

	moveit::planning_interface::MoveGroup::Plan my_plan;

	float _tray_location_x, _tray_location_y, _tray_location_z, _tray_length;

	double test_x, test_y, test_z;

	std::vector<double> home_joint_values, base_link_end_values, return_home_joint_values;
	void initialSetup();
	void goHome();
	float getRandomValue();

public:
	PickAndPlace(ros::NodeHandle n, double* initialPositions, double _z_offset_from_part, double* part_location, float tray_length = 0.2);
	void performPickAndPlace();
	void pickNextPart();
	void place();
	void pickNextPart(geometry_msgs::Vector3 obj_pose);
};
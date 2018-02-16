#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

class PickAndPlace {
private:
	ros::NodeHandle nh_;
	geometry_msgs::Pose _homePose;

	moveit::planning_interface::MoveGroup _manipulatorgroup;
	tf::StampedTransform _tray_to_world_;

	tf::TransformListener tf_tray_to_world;

	moveit::planning_interface::MoveGroup::Plan my_plan;

	float _tray_location_x, _tray_location_y, _tray_location_z;

	void attainPosition(float x, float y, float z);

	std::vector<double> home_joint_values;
	void initialSetup();

public:
	PickAndPlace(ros::NodeHandle n);
	void performPickAndPlace();
};
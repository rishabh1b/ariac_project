#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/StorageUnit.h>

struct Bin {
	double resolution, orig_start_x, orig_start_y, start_x, start_y, start_z, z_offset_from_part;
	int curr_step;
	bool is_res_set, is_offset_set;
	std::string bin_name;

	Bin() {
		start_x = 0;
	    start_y = 0;
		start_z = 0.72;
		bin_name = "bin";
		curr_step = 0;
		is_res_set = false;
	}

	bool checkOverflow() {
		return ((curr_step + 1) > (0.6 - std::abs(resolution)) / std::abs(resolution) - 1);
	}
	
	Bin(std::string bin, double start_loc_x, double start_loc_y, double start_loc_z) {
		orig_start_x = start_loc_x;
		orig_start_y = start_loc_y;
		start_x = start_loc_x;
		start_y = start_loc_y;
		start_z = start_loc_z;
		bin_name = bin;
		curr_step = 0;
		is_res_set = false;
	}
	void incStep() {
		curr_step += 1;
		start_x = start_x - std::abs(resolution);
		start_y = start_y + resolution;
		if (curr_step > (0.6 - std::abs(resolution)) / std::abs(resolution) - 1) {
			start_x = orig_start_x;
			start_y = orig_start_y + 0.4;
			resolution = -resolution;
			curr_step = 0;
		}
	}	
};

class PickAndPlace {
private:
	ros::NodeHandle nh_;
	geometry_msgs::Pose _homePose;
	geometry_msgs::Quaternion _home_orientation, _tray_orientation;
	geometry_msgs::Vector3 _home_position;
	geometry_msgs::Point _home_pt;
	osrf_gear::VacuumGripperControl gripper_srv;
	osrf_gear::GetMaterialLocations mat_location_srv;
	ros::ServiceClient gripper_client, mat_location_client;
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
	
	std::map<std::string, std::string> partLocation;
	std::map<std::string, Bin> binMap;

	void initialSetup();
	void goHome();
	void goHome2();
	void setHome(); 
	float getRandomValue();
	void fillPartLocation(std::string mat_type);

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
	bool pickNextPartBin(std::string partType);

    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr& msg);
};
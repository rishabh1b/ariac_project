#include "pick_and_place/PickAndPlace.h"
#include "localisation/request_logical_pose.h"
#include "osrf_gear/AGVControl.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "floor_manager");
	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");

	double initialjoints[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double z_offset_from_part, x_test, y_test, z_test; // Some Distance offset in Z before activating suction

	private_node_handle.getParam("joint1", initialjoints[1]);
	private_node_handle.getParam("joint2", initialjoints[2]);
	private_node_handle.getParam("joint3", initialjoints[3]);
	private_node_handle.getParam("joint4", initialjoints[4]);
	private_node_handle.getParam("joint5", initialjoints[5]);
	private_node_handle.getParam("joint6", initialjoints[6]);
	private_node_handle.getParam("z_offset", z_offset_from_part);
	private_node_handle.getParam("x_test", x_test);
	private_node_handle.getParam("y_test", y_test);
	private_node_handle.getParam("z_test", z_test);

	double part_location[3] = {x_test, y_test, z_test};

    // ROS_INFO_STREAM("Initial Joint 6: " << initialjoints[6] << "\n");

	PickAndPlace pickPlace(n, initialjoints, z_offset_from_part, part_location);

	ros::ServiceClient nextPointclient = n.serviceClient<localisation::request_logical_pose>("logical_camera_server");
    localisation::request_logical_pose pointsrv;
    pointsrv.request.request_msg = true;


	ros::spinOnce();

	geometry_msgs::Vector3 obj_pose;

	while (ros::ok()) {
		if (nextPointclient.call(pointsrv))
	   {
	   	if (pointsrv.response.order_completed)
	   		break;

	    obj_pose = pointsrv.response.position;

	    pickPlace.pickNextPart(obj_pose);
	    pickPlace.place();
	   }
	   else
	  {
	    ROS_WARN("Service Not Ready");
	  }

	}

	ros::ServiceClient submissionclient = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
	osrf_gear::AGVControl subsrv;
    subsrv.request.kit_type = "order_0_kit_0";
    submissionclient.call(subsrv);
}
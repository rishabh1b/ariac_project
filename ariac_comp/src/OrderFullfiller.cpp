#include "ariac_comp/OrderFullfiller.h"

OrderFullfiller::OrderFullfiller(ros::NodeHandle n) {
     this->n = n;
	 nextPartclient = n.serviceClient<ariac_comp::request_next_pose>("logical_camera_server");
	 incrementclient = n.serviceClient<std_srvs::Trigger>("incrementPart");
	 // highPriorityServer = n.advertiseService("high_priority_om_server", &FloorManager::highPriorityService, this);
	 pointsrv.request.request_msg = true;
	 kit_num = 0;
	 usedAGV2 = false;
	 highPriorityOrderReceived = false;
}

bool OrderFullfiller::manage(PickAndPlace& pickPlace) {
	// bool conveyorPartAvailable;

	if (nextPartclient.call(pointsrv))
   {
   		// if (highPriorityOrderReceived) {
    	//    	highPriorityOrderReceived = false;
    	//    	pickPlace.dropPartSafely(usedAGV2);
    	//    	usedAGV2 = !usedAGV2;
    	// }

   	if (pointsrv.response.noPartFound)
   		return false;
   	
   	if (pointsrv.response.order_completed)
   		return false;

    obj_pose = pointsrv.response.position;
    target_pose = pointsrv.response.tgtposition;
    obj_orientation = pointsrv.response.orientation;
    target_orientation = pointsrv.response.tgtorientation;

    if (pointsrv.response.conveyorPart) {
    	if (!pickPlace.pickPlaceNextPartConveyor(obj_pose, target_pose, target_orientation, !usedAGV2))
    		return true;
    }

    else if (!pickPlace.pickAndPlace(obj_pose, obj_orientation, target_pose, target_orientation, !usedAGV2)) {
    	   return true;
    }
    
   }
   else
  {
    ROS_WARN("Service Not Ready");
  }

  incrementclient.call(incPart);

    std::stringstream ss;
    ss << "order_0_kit_" << kit_num;
    std::string kit_type = ss.str();

    if (!usedAGV2 && incPart.response.success) {
		submissionclient = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
		usedAGV2 = true;
		kit_num += 1;
    }
	else if (usedAGV2 && incPart.response.success){
		submissionclient = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
		usedAGV2 = false;
		kit_num += 1;
	}

    subsrv.request.kit_type = kit_type;
    if (incPart.response.success)
    	submissionclient.call(subsrv);

    return true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "floor_manager");
	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");

	double initialjoints[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double z_offset_from_part, x_test, y_test, z_test; // Some Distance offset in Z before activating suction
    float tray_length;

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
	private_node_handle.getParam("tray_length", tray_length);

	double part_location[3] = {x_test, y_test, z_test};

    // ROS_INFO_STREAM("Initial Joint 6: " << initialjoints[6] << "\n");

	PickAndPlace pickPlace(n, initialjoints, z_offset_from_part, part_location, tray_length);

	OrderFullfiller orderFullfiller(n);

	ROS_WARN("Starting Pick and Place");

    while (ros::ok()) {
    	if (!orderFullfiller.manage(pickPlace))
    		break;
    }
    return 0;
}
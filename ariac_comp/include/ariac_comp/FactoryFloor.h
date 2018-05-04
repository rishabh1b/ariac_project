#include <osrf_gear/GetMaterialLocations.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <osrf_gear/StorageUnit.h>

struct Bin {
	double x_resolution, y_resolution, orig_start_x, orig_start_y, start_x, start_y, start_z, z_offset_from_part;
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
		return ((curr_step + 1) > (0.6 - std::abs(y_resolution)) / std::abs(y_resolution) - 1);
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
		start_x = start_x - std::abs(x_resolution);
		start_y = start_y + y_resolution;
		if (curr_step > (0.6 - std::abs(y_resolution)) / std::abs(y_resolution) - 1) {
			start_x = orig_start_x;
			start_y = orig_start_y + 0.4;
			y_resolution = -y_resolution;
			curr_step = 0;
		}
	}

	void setResolution(double res) {
		x_resolution = res;
		y_resolution = res;
	}
};	

class FactoryFloor {
	private :
		ros::ServiceClient mat_location_client;
		std::map<std::string, std::string> partLocation;
		osrf_gear::GetMaterialLocations mat_location_srv;
	    void fillPartLocation(std::string mat_type) {
			  mat_location_srv.request.material_type = mat_type;
			  mat_location_client.call(mat_location_srv);
			  if (mat_location_srv.response.storage_units.size() > 0 && mat_location_srv.response.storage_units[0].unit_id.compare("belt") != 0) {
			    partLocation.insert(std::make_pair(mat_type, mat_location_srv.response.storage_units[0].unit_id));
			  }
			}

	public :
		FactoryFloor(ros::NodeHandle nh_) {
		  mat_location_client = nh_.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
		  fillPartLocation("gear_part");
		  fillPartLocation("piston_rod_part");
		  fillPartLocation("gasket_part");
		  fillPartLocation("disk_part");
		  fillPartLocation("pulley_part");
		  ROS_INFO("Done Populating");
		}

	    std::map<std::string, std::string>  getPartLocations() {
	    	return partLocation;
	    }
};

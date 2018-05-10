#include <osrf_gear/GetMaterialLocations.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <osrf_gear/StorageUnit.h>

struct Bin {
	double x_resolution, y_resolution, orig_start_x, orig_start_y;
	double start_x, start_y, start_z, z_offset_from_part;
	double origin_x, origin_y;
	int curr_step;
	bool is_res_set, is_offset_set, is_part_found, directionFlipped;
	std::string bin_name;

	Bin() {
		start_x = 0;
	    start_y = 0;
		start_z = 0.72;
		bin_name = "bin";
		curr_step = 0;
		is_res_set = false;
	}

	bool checkOverflowY() {
		return (std::abs(start_y + y_resolution - orig_start_y) > 0.45);
	}

	bool checkOverflowX() {
		return (std::abs(start_x - orig_start_x - x_resolution) > 0.45);
	}

	bool setPartFound() {
		is_part_found = true;
		origin_x = start_x;
		origin_y = orig_start_y;
		// x_resolution = (curr_step + 1) * x_resolution;
		// y_resolution = (curr_step + 1) * y_resolution;
		incStep();
		curr_step = 0;
		directionFlipped = false;
	}

	Bin(std::string bin, double start_loc_x, double start_loc_y, double start_loc_z) {
		orig_start_x = start_loc_x;
		orig_start_y = start_loc_y;
		start_x = start_loc_x;
		start_y = start_loc_y;
		start_z = start_loc_z;
		bin_name = bin;
		curr_step = 0;
		origin_x = 0;
		origin_y = 0;
		is_res_set = false;
		is_part_found = false;
		directionFlipped = false;
	}

	void incStep() {
		if (checkOverflowX() && !is_part_found) {
			ROS_WARN(" Going For Other Diagonal ");
			start_x = orig_start_x;
			start_y = orig_start_y + 0.38;
			directionFlipped = true;
		}

		else if (checkOverflowX()) {
			start_x = orig_start_x;
			start_y = orig_start_y;
		} else if (checkOverflowY() && is_part_found){
			start_x = start_x - x_resolution;
			start_y = origin_y;
			// curr_step = 0;
		}
		else if (!is_part_found){
			start_x = start_x - x_resolution;
			if (!directionFlipped)
				start_y = start_y + y_resolution;
			else
				start_y = start_y - y_resolution;
			curr_step += 1;
		}
		else {
			start_y = start_y + y_resolution;
			// curr_step += 1;
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

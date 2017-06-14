#ifndef PARTS_GRASP_H_
#define PARTS_GRASP_H_

#include <vector>
#include <tf/transform_datatypes.h>

// using namespace std;

std::map<std::string,std::vector<tf::Vector3> > default_search_locations;

void init_default_search_locations(){
	default_search_locations["piston_rod_part"] = std::vector<tf::Vector3>();
	default_search_locations["gasket_part"] = std::vector<tf::Vector3>();
	default_search_locations["pulley_part"] = std::vector<tf::Vector3>();
	default_search_locations["gear_part"] = std::vector<tf::Vector3>();
	default_search_locations["disk_part"] = std::vector<tf::Vector3>();

	default_search_locations["piston_rod_part"].push_back(tf::Vector3(0.2200, 0.2200, 0));
	default_search_locations["piston_rod_part"].push_back(tf::Vector3(0.100, 0.100, 0));
	default_search_locations["piston_rod_part"].push_back(tf::Vector3(0.2200, 0.0700, 0));
	default_search_locations["piston_rod_part"].push_back(tf::Vector3(0.0700, 0.2200, 0));

	default_search_locations["disk_part"].push_back(tf::Vector3(0.2300, 0.2300, 0));
	default_search_locations["disk_part"].push_back(tf::Vector3(0.0600, 0.0600, 0));
	default_search_locations["disk_part"].push_back(tf::Vector3(0.2300, 0.0600, 0));
	default_search_locations["disk_part"].push_back(tf::Vector3(0.0600, 0.2300, 0));

	default_search_locations["pulley_part"].push_back(tf::Vector3(0.15, 0.15, 0));

	default_search_locations["gear_part"].push_back(tf::Vector3(0.2300, 0.2300, 0));
	default_search_locations["gear_part"].push_back(tf::Vector3(0.1500, 0.1500, 0));
	default_search_locations["gear_part"].push_back(tf::Vector3(0.0700, 0.0700, 0));
	default_search_locations["gear_part"].push_back(tf::Vector3(0.2300, 0.0700, 0));
	default_search_locations["gear_part"].push_back(tf::Vector3(0.0700, 0.2300, 0));

}

// disk_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));
// disk_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));
// disk_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));
// disk_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));


// gear_primary_loc.push_back(tf::Vector3(0.2, 0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(0.2, 0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(0.2, 0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(0.2, 0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(0.12, 0.12, 0));
// gear_primary_loc.push_back(tf::Vector3(0.12, 0.12, 0));
// gear_primary_loc.push_back(tf::Vector3(0.12, 0.12, 0));
// gear_primary_loc.push_back(tf::Vector3(0.12, 0.12, 0));



// pulley_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));
// pulley_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));
// pulley_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));
// pulley_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));

// piston_primary_loc.push_back(tf::Vector3(0.1962, 0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1962, 0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1962, 0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1962, 0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1132, 0.1132, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1132, 0.1132, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1132, 0.1132, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1132, 0.1132, 0));

#endif
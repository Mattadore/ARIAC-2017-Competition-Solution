#ifndef PARTS_GRASP_H_
#define PARTS_GRASP_H_

#include <vector>
#include <tf/transform_datatypes.h>

// using namespace std;

std::vector<tf::Vector3> disk_primary_loc;
std::vector<tf::Vector3> gear_primary_loc;
std::vector<tf::Vector3> piston_primary_loc;
std::vector<tf::Vector3> pulley_primary_loc;


void init_search_locations (){
	piston_primary_loc.push_back(tf::Vector3(-0.2200, -0.2200, 0));
	piston_primary_loc.push_back(tf::Vector3(-0.100, -0.100, 0));
	piston_primary_loc.push_back(tf::Vector3(-0.2200, -0.0700, 0));
	piston_primary_loc.push_back(tf::Vector3(-0.0700, -0.2200, 0));

	disk_primary_loc.push_back(tf::Vector3(-0.2300, -0.2300, 0));
	disk_primary_loc.push_back(tf::Vector3(-0.0600, -0.0600, 0));
	disk_primary_loc.push_back(tf::Vector3(-0.2300, -0.0600, 0));
	disk_primary_loc.push_back(tf::Vector3(-0.0600, -0.2300, 0));

	pulley_primary_loc.push_back(tf::Vector3(-0.15, -0.15, 0));

	gear_primary_loc.push_back(tf::Vector3(-0.2300, -0.2300, 0));
	gear_primary_loc.push_back(tf::Vector3(-0.1500, -0.1500, 0));
	gear_primary_loc.push_back(tf::Vector3(-0.0700, -0.0700, 0));
	gear_primary_loc.push_back(tf::Vector3(-0.2300, -0.0700, 0));
	gear_primary_loc.push_back(tf::Vector3(-0.0700, -0.2300, 0));

}

// disk_primary_loc.push_back(tf::Vector3(-0.15, -0.15, 0));
// disk_primary_loc.push_back(tf::Vector3(-0.15, 0.15, 0));
// disk_primary_loc.push_back(tf::Vector3(0.15, -0.15, 0));
// disk_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));


// gear_primary_loc.push_back(tf::Vector3(-0.2, -0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(-0.2, 0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(0.2, -0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(0.2, 0.2, 0));
// gear_primary_loc.push_back(tf::Vector3(-0.12, -0.12, 0));
// gear_primary_loc.push_back(tf::Vector3(-0.12, 0.12, 0));
// gear_primary_loc.push_back(tf::Vector3(0.12, -0.12, 0));
// gear_primary_loc.push_back(tf::Vector3(0.12, 0.12, 0));



// pulley_primary_loc.push_back(tf::Vector3(-0.15, -0.15, 0));
// pulley_primary_loc.push_back(tf::Vector3(0.15, -0.15, 0));
// pulley_primary_loc.push_back(tf::Vector3(-0.15, 0.15, 0));
// pulley_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));

// piston_primary_loc.push_back(tf::Vector3(-0.1962, -0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(-0.1962, 0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1962, -0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1962, 0.1962, 0));
// piston_primary_loc.push_back(tf::Vector3(-0.1132, -0.1132, 0));
// piston_primary_loc.push_back(tf::Vector3(-0.1132, 0.1132, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1132, -0.1132, 0));
// piston_primary_loc.push_back(tf::Vector3(0.1132, 0.1132, 0));
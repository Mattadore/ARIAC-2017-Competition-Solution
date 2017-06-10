#ifndef PARTS_GRASP_H_
#define PARTS_GRASP_H_

#include <vector>
#include <tf/transform_datatypes.h>

using namespace std;

std::vector<tf::Vector3> disk_primary_loc;
disk_primary_loc.push_back(tf::Vector3(-0.21, -0.21, 0));
disk_primary_loc.push_back(tf::Vector3(0.21, -0.21, 0));
disk_primary_loc.push_back(tf::Vector3(-0.21, 0.21, 0));
disk_primary_loc.push_back(tf::Vector3(0.21, 0.21, 0));
disk_primary_loc.push_back(tf::Vector3(-0.1251, -0.1251, 0));
disk_primary_loc.push_back(tf::Vector3(0.1251, -0.1251, 0));
disk_primary_loc.push_back(tf::Vector3(-0.1251, 0.1251, 0));
disk_primary_loc.push_back(tf::Vector3(0.1251, 0.1251, 0));

std::vector<tf::Vector3> gear_primary_loc;
gear_primary_loc.push_back(tf::Vector3(-0.24, -0.24, 0));
gear_primary_loc.push_back(tf::Vector3(0.24, -0.24, 0));
gear_primary_loc.push_back(tf::Vector3(-0.24, 0.24, 0));
gear_primary_loc.push_back(tf::Vector3(0.24, 0.24, 0));
gear_primary_loc.push_back(tf::Vector3(-0.1693, -0.1693, 0));
gear_primary_loc.push_back(tf::Vector3(0.1693, -0.1693, 0));
gear_primary_loc.push_back(tf::Vector3(-0.1693, 0.1693, 0));
gear_primary_loc.push_back(tf::Vector3(0.1693, 0.1693, 0));
gear_primary_loc.push_back(tf::Vector3(-0.0986, -0.0986, 0));
gear_primary_loc.push_back(tf::Vector3(0.0986, -0.0986, 0));
gear_primary_loc.push_back(tf::Vector3(-0.0986, 0.0986, 0));
gear_primary_loc.push_back(tf::Vector3(0.0986, 0.0986, 0));

std::vector<tf::Vector3> pulley_primary_loc;
pulley_primary_loc.push_back(tf::Vector3(-0.15, -0.15, 0));
pulley_primary_loc.push_back(tf::Vector3(0.15, -0.15, 0));
pulley_primary_loc.push_back(tf::Vector3(-0.15, 0.15, 0));
pulley_primary_loc.push_back(tf::Vector3(0.15, 0.15, 0));

std::vector<tf::Vector3> piston_primary_loc;
piston_primary_loc.push_back(tf::Vector3(-0.2377, -0.2377, 0));
piston_primary_loc.push_back(tf::Vector3(0.2377, -0.2377, 0));
piston_primary_loc.push_back(tf::Vector3(-0.2377, 0.2377, 0));
piston_primary_loc.push_back(tf::Vector3(0.2377, 0.2377, 0));
piston_primary_loc.push_back(tf::Vector3(-0.167, -0.167, 0));
piston_primary_loc.push_back(tf::Vector3(0.167, -0.167, 0));
piston_primary_loc.push_back(tf::Vector3(-0.167, 0.167, 0));
piston_primary_loc.push_back(tf::Vector3(0.167, 0.167, 0));
piston_primary_loc.push_back(tf::Vector3(-0.0963, -0.0963, 0));
piston_primary_loc.push_back(tf::Vector3(0.0963, -0.0963, 0));
piston_primary_loc.push_back(tf::Vector3(-0.0963, 0.0963, 0));
piston_primary_loc.push_back(tf::Vector3(0.0963, 0.0963, 0));

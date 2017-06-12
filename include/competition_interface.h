#ifndef COMPETITION_INTERFACE_H_
#define COMPETITION_INTERFACE_H_

#include <algorithm>
#include <vector>
#include <utility>
#include <string>
#include <map>
#include <list>
#include <cstdio>
#include <memory>
#include <cmath>

#include <ros/ros.h>

#include <include/object_manager.h>
#include <include/utility.h>

#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/AGVControl.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <keyboard/Key.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer_server.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#include <control_msgs/JointTrajectoryControllerState.h>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

struct CompetitionInterface { //exists for this one node, everything is static

	//initialization
	template<class M>
	static void subscribe(std::string name,uint32_t size,void(*fp)(M)) {
		subscriptions[name] = nodeptr->subscribe(name,size,fp);
	}

	static void initialize_interface(ros::NodeHandle * _nodeptr) {
		nodeptr = _nodeptr;
		AGV_info[0] = AGV_data(1,"agv1_load_point_frame","/ariac/agv1");
		AGV_info[1] = AGV_data(2,"agv2_load_point_frame","/ariac/agv2");
		AGV_status_lookup["delivering"] = AGV_DELIVERING;
		AGV_status_lookup["delivered"] = AGV_DELIVERED;
		AGV_status_lookup["returning"] = AGV_RETURNING;
		AGV_status_lookup["ready_to_deliver"] = AGV_READY_TO_DELIVER;
		AGV_status_lookup["preparing_to_deliver"] = AGV_PREPARING_TO_DELIVER;
		dropped = false;
		add_subscriptions();


	}

	static void add_subscriptions() {
		subscribe("/ariac/current_score", 10, CompetitionInterface::current_score_callback);
		subscribe("/ariac/competition_state", 10, CompetitionInterface::competition_callback);
		subscribe("/ariac/orders", 100, CompetitionInterface::order_callback);
		//subscribe("/ariac/joint_states", 10, CompetitionInterface::joint_state_callback);
		//subscribe("/ariac/proximity_sensor_1_change", 10, proximity_sensor_callback);
		//subscribe("/ariac/break_beam_1_change", 10, break_beam_callback);
		subscribe("/ariac/logical_camera_1", 10, CompetitionInterface::logical_camera_callback);
		subscribe("/ariac/quality_control_sensor_1", 1, CompetitionInterface::faulty_part_detector_1_callback);
		subscribe("/ariac/quality_control_sensor_2", 1, CompetitionInterface::faulty_part_detector_2_callback);
		//subscribe("/ariac/laser_profiler_1", 10, laser_profiler_callback);
		subscribe("/ariac/gripper/state", 1, CompetitionInterface::vacuum_gripper_callback);
		subscribe("/ariac/agv1/state", 1, CompetitionInterface::AGV1_callback);
		subscribe("/ariac/agv2/state", 1, CompetitionInterface::AGV2_callback);
		subscribe("/keyboard/keydown", 10, CompetitionInterface::key_down_callback);
	}

	//---------------action methods
	static void toggle_vacuum(bool _enabled) {
		ROS_INFO("Toggle requested: %d", (int)_enabled);
		if (CompetitionInterface::get_state(GRIPPER_ENABLED) == (_enabled+1)) return;
		ROS_INFO("Toggle attempted");
		ros::ServiceClient vacuum_client = nodeptr->serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
		osrf_gear::VacuumGripperControl srv;
		srv.request.enable = _enabled;
		vacuum_client.call(srv);
		if (!srv.response.success) {
			ROS_ERROR("Unable to enable gripper");
		}
	}

	static void toggle_fake_vacuum(bool _enabled) {
		ros::ServiceClient vacuum_client = nodeptr->serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/fake_control");
		osrf_gear::VacuumGripperControl srv;
		srv.request.enable = _enabled;
		vacuum_client.call(srv);
		if (!srv.response.success) {
			ROS_ERROR("Unable to enable fake gripper");
		}
	}

	//command methods
	static void send_AGV(char AGV_num,std::string kit_type) {
		ros::ServiceClient agv_client = nodeptr->serviceClient<osrf_gear::AGVControl>(AGV_info[AGV_num].topic.c_str());
		osrf_gear::AGVControl srv;
		srv.request.kit_type = kit_type;
		agv_client.call(srv);
	}

	static void start_competition() {
		ros::ServiceClient start_client = nodeptr->serviceClient<std_srvs::Trigger>("/ariac/start_competition");
		std_srvs::Trigger srv;
		start_client.call(srv);
		if (!srv.response.success) {
			ROS_ERROR_STREAM("Unable to start competition:" << srv.response.message);
		}
	}

	static void end_competition() {
		ros::ServiceClient start_client = nodeptr->serviceClient<std_srvs::Trigger>("/ariac/end_competition");
		std_srvs::Trigger srv;
		start_client.call(srv);
		if (!srv.response.success) {
			ROS_ERROR_STREAM("Unable to end competition:" << srv.response.message);
		}
	}

	static std::vector<std::string> get_material_locations(std::string material_name) {//returns bin names
		ros::ServiceClient material_client = nodeptr->serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
		osrf_gear::GetMaterialLocations srv;
		srv.request.material_type = material_name;
		material_client.call(srv);
		std::vector<std::string> bin_names;
		for (osrf_gear::StorageUnit & unit : srv.response.storage_units) {
			if (unit.unit_id != "belt") { //sry
				bin_names.push_back(unit.unit_id);
			}
		}
		return bin_names;
	}
	static void set_conveyor(float percent) {
		ros::ServiceClient conveyor_client = nodeptr->serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
		osrf_gear::ConveyorBeltControl srv;
		srv.request.state.power = percent;
		conveyor_client.call(srv);
		if (!srv.response.success) {
			ROS_ERROR("Unable to command conveyor");
		}
	}
	static void set_arm_region(char new_region) {
		arm_region = new_region;
	}
	static char get_arm_region() {
		return arm_region;
	}
	//access methods
	static unsigned char get_state(unsigned char index) {
		return state[index];
	}

	static double get_current_score() {
		return current_score;
	}

	static unsigned char get_agv_state(unsigned char agv_id) {
		if (agv_id == 1) {
			return AGV_info[0].status;
		}
		else if (agv_id == 2) {
			return AGV_info[1].status;
		}
		else {
			ROS_ERROR("Invalid AGV");
			return 0;
		}
	}

	static std::vector<osrf_gear::Order> & get_orders() {
		return received_orders;
	}

	static osrf_gear::LogicalCameraImage get_quality_control_msg(char AGV) {
		if (!((AGV == 1) || (AGV == 2))) {
			ROS_ERROR("Invalid AGV: AGV %d",(int)AGV);
		}
		return last_faulty_part_scan[AGV-1];
	}

	//below methods are probably not useful anymore
	// static unsigned char get_state_diff(unsigned char index) {
	// 	return state_diff[index];
	// }

	// static unsigned char get_state_old(unsigned char index) {
	// 	return state_old[index];
	// }

	// static unsigned char get_state_if_diff(unsigned char index)[ 
	// 	return state_if_diff[index];
	// }

	static bool part_dropped() {
		return dropped;
	}

protected:

	//---------------callbacks (should not invoke any actions or action methods)
	static void competition_callback(const std_msgs::String::ConstPtr msg) {
		if (msg->data == "done") {
			state[COMPETITION_STATE] = COMPETITION_DONE;
		}
		else if (msg->data == "go") {
			state[COMPETITION_STATE] = COMPETITION_GO;
		}
		else if (msg->data == "ready") {
			state[COMPETITION_STATE] = COMPETITION_READY;
		}
		else if (msg->data == "end_game") {
			state[COMPETITION_STATE] = COMPETITION_END_GAME;
		}
		else if (msg->data == "init") {
			state[COMPETITION_STATE] = COMPETITION_INIT;
		}
	}

	static void current_score_callback(const std_msgs::Float32::ConstPtr msg) {
		if (msg->data != current_score)
		{
			ROS_INFO_STREAM("Current score: " << msg->data);
		}
		current_score = msg->data;
	}

	static void AGV1_callback(const std_msgs::String::ConstPtr msg) {
		AGV_info[0].status = AGV_status_lookup[msg->data];
	}

	static void AGV2_callback(const std_msgs::String::ConstPtr msg) {
		AGV_info[1].status = AGV_status_lookup[msg->data];
	}


	static void vacuum_gripper_callback(const osrf_gear::VacuumGripperState::ConstPtr msg) {
		if ((msg->attached == false) && (state[GRIPPER_ATTACHED] == BOOL_TRUE)) {
			//only triggers if part was being held a moment ago
			if (msg->enabled == true) {
				dropped = true; //part was dropped accidentally
			}
		}
		state[GRIPPER_ATTACHED] = (msg->attached) ? BOOL_TRUE : BOOL_FALSE;
		state[GRIPPER_ENABLED] = (msg->enabled) ? BOOL_TRUE : BOOL_FALSE;
		if (state[GRIPPER_ATTACHED] == BOOL_TRUE) {
			if (ObjectTracker::get_held_object() != ObjectTracker::get_interested_object()) {
				if (ObjectTracker::get_held_object() == "") {
					ObjectTracker::pick_up();
				}
			}
		}
		else {
			if (ObjectTracker::get_held_object() != "") {
				ObjectTracker::drop_off();
			}
		}
		if (state[GRIPPER_ENABLED] == BOOL_FALSE) {
			dropped = false;
		}
	}

	static void key_down_callback(const keyboard::Key::ConstPtr msg) {
		//ros::NodeHandle node;
		tf::StampedTransform tray_base;
		switch (msg->code) {
			case keyboard::Key::KEY_i:
				if (state[COMPETITION_STATE] == COMPETITION_INIT) { //only start if not running
					start_competition();
				}
				else {
					end_competition();
				}
				break;
			case keyboard::Key::KEY_r:
				toggle_vacuum(false);
				break;
			case keyboard::Key::KEY_d:
				//ROS_INFO("Current pipeline status is: %d");
				//send_arm_to_zero_state(); // do something else lol
				break;
			case keyboard::Key::KEY_g:
				toggle_vacuum(true);
				break;
			case keyboard::Key::KEY_f:
				toggle_fake_vacuum(true);
				break;
		}
	}

	static void order_callback(const osrf_gear::Order::ConstPtr order_msg) {
		ROS_INFO_STREAM("Received order:\n" << *order_msg);
		//recheck_assignments = true; //check if AGV tray assignments have changed
		received_orders.push_back(*order_msg);
	}

	static void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr image_msg) {

	}

	static void faulty_part_detector_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr image_msg) {
		last_faulty_part_scan[0] = *image_msg;
	}

	static void faulty_part_detector_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr image_msg) {
		last_faulty_part_scan[1] = *image_msg;
	}


	static std::vector<osrf_gear::Order> received_orders; //just an order list
	static std::map<std::string,ros::Subscriber> subscriptions;
	static osrf_gear::LogicalCameraImage last_faulty_part_scan[2];
	static std::map<std::string,char> AGV_status_lookup;
	static double current_score;
	static sensor_msgs::JointState current_joint_state;
	static unsigned char state[NUM_STATES]; //holds state machine info
	static AGV_data AGV_info[2]; //basic agv information
	static char arm_region;
	static bool dropped;
	static std::string object_held;
	static std::string object_interested;

	// static unsigned char state_diff[NUM_STATES]; //holds diff info
	// static unsigned char state_old[NUM_STATES]; //holds old state machine info
	// static unsigned char state_if_diff[NUM_STATES]; //holds old state machine info
	static ros::NodeHandle * nodeptr;
	CompetitionInterface() {};
};

// THIS OBJECT FILE OWNS THE DEFINITIONS FOR COMPETITIONINTERFACE
std::vector<osrf_gear::Order> CompetitionInterface::received_orders; //just an order list
std::map<std::string,ros::Subscriber> CompetitionInterface::subscriptions;
osrf_gear::LogicalCameraImage CompetitionInterface::last_faulty_part_scan[2];
std::map<std::string,char> CompetitionInterface::AGV_status_lookup;
double CompetitionInterface::current_score;
sensor_msgs::JointState CompetitionInterface::current_joint_state;
unsigned char CompetitionInterface::state[NUM_STATES]; //holds state machine info
AGV_data CompetitionInterface::AGV_info[2]; //basic agv information
char CompetitionInterface::arm_region;
bool CompetitionInterface::dropped;
std::string CompetitionInterface::object_held;
std::string CompetitionInterface::object_interested;

// unsigned char state_diff[NUM_STATES]; //holds diff info
// unsigned char state_old[NUM_STATES]; //holds old state machine info
// unsigned char state_if_diff[NUM_STATES]; //holds old state machine info
ros::NodeHandle * CompetitionInterface::nodeptr;

//TODO: neat way to deal with multi bin indexing
//TODO: for each part type, keep a list of bin_id_offsets

#endif
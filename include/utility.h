#ifndef UTILITY_H_
#define UTILITY_H_

//utility file, just generic methods/data/types
//TODO: clean this up a bit

#include <vector>
#include <string>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>


const double AGV_acceptable_range_ = 0.03; //acceptable radius (magic number, but should work fine?)
const double grab_height_compensation = 0.1;
const tf::Vector3 AGV_1_start_pos = tf::Vector3(4.25,3.30,0.21);
const tf::Vector3 AGV_2_start_pos = tf::Vector3(4.25,-3.00,0.21);
//planning an end effector to its pose but with this x and z value creates retraction poses
const tf::Vector3 retraction_offset = tf::Vector3(-0.16,0,1.3); 
const tf::Quaternion identity(0,0,0,1);
const double actuator_limit = 2.1;
const double intermediate_limit = actuator_limit-0.8;
# define M_PI       3.14159265358979323846  //just in case
const double grab_palm_offset = -0.003; //used to determine where to grab parts
const double camera_noise_tolerance = 0.006; //used to determine location of parts
const double conveyor_speed = 0.2; // m/s'

//whatever time moveit takes to figure itself out
const ros::Duration transition_delay(0.0);

const double trashcan_drop_offset = M_PI/4.0;
const double intermediate_configuration_data[2][7] {
{-0.8, (3.0/2.0)*M_PI, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268},
{0.35, 1.5755329961346092, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268}};

const double trashcan_configuration_data[2][7] {
{-0.8, (3.0/2.0)*M_PI-trashcan_drop_offset, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268},
{0.35, 1.5755329961346092+trashcan_drop_offset, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268}};

//TODO: conveyor region and bin pullup grab region
const double conveyor_configuration_data[7]
{0, 0, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268};

const double pickup_configuration_data[7]
{-0.8, 0, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268};

const double true_center = -0.15;

enum AGV_states {
	AGV_NONE,
	AGV_DELIVERING,
	AGV_DELIVERED,
	AGV_PREPARING_TO_DELIVER, //really short state
	AGV_READY_TO_DELIVER, //default, when the agv is *here*
	AGV_RETURNING
};

enum simple_regions {
	SIMPLE_REGION_NONE,
	REGION_AGV1,
	REGION_AGV2,
	REGION_BINS, //treat bins and conveyor as same for now
	REGION_CONVEYOR,
	REGION_BIN_GRAB //technically unsafe to have just "bin" region
};

enum status_states {
	BOOLEAN_NONE,
	BOOL_FALSE,
	BOOL_TRUE,
	ERROR_STATE
};

// enum plan_types {
// 	PLAN_NONE,
// 	PULLUP_PLAN,
// 	FORWARD_PLAN,
// 	GRAB_PLAN,
// 	RETRACT_PLAN,
// 	DROP_PLAN,
// 	EXCEPT_PLAN, //plan for when same name but different target, used in weird cases
// 	NO_PLAN,
// 	DEBUG_PLAN,
// 	PLANNING
// };

enum competition_states {
	COMPETITION_NONE,
	COMPETITION_END_GAME,
	COMPETITION_GO,
	COMPETITION_DONE,
	COMPETITION_INIT,
	COMPETITION_READY
};

enum state_types {
	STATE_NONE,
	GRIPPER_ATTACHED,
	GRIPPER_ENABLED,
	COMPETITION_STATE,
	MOVING,
	PART_HELD,
	TERMINATED,
	AGV_1_STATE,
	AGV_2_STATE,
	NUM_STATES
};


/*
enum flag_types {
	FLAG_nullptr,
	NUM_FLAGS
};*/

enum pipeline_status {
	PIPELINE_nullptr,
	PIPELINE_NONE,
	PIPELINE_WAITING,
	PIPELINE_PROCESSING,
	PIPELINE_COMPLETE
};

enum configuration_name {
	NEGATIVE_CONFIGURATION,
	POSITIVE_CONFIGURATION
};


//data structs

struct AGV_data {
	std::string num;
	std::string base_frame;
	std::string topic;
	char status;
	AGV_data(char num_, std::string base_frame_, std::string topic_) :
	num({num_,'\0'}), base_frame(base_frame_),
	topic(topic_) {status = AGV_READY_TO_DELIVER;}
	AGV_data() {} //so I don't have to put this at the start of main construction
};

struct arm_action {
	using Ptr = std::shared_ptr<arm_action>; 
	char planning_status = PIPELINE_NONE;
	char execution_status = PIPELINE_NONE;
	char region = SIMPLE_REGION_NONE;
	//unsigned char action_type;
	moveit::planning_interface::MoveGroup::Plan * plan = nullptr;
	tf::Pose trajectory_end;
	double end_delay = 0.0;
	bool true_pose = false;
	bool vacuum_enabled = false;
	bool pick_part = false;
	bool use_intermediate = false;	//prepares to go to an intermediate pose
	bool use_trash = false; //prepares to drop a faulty item
	bool use_conveyor = false; //prepares to grab off conveyor
	arm_action::Ptr parent = nullptr;
	moveit_msgs::Constraints * constraints = nullptr;
	//maybe add what the state is here
	ros::Duration get_execution_time() {
		if (plan != nullptr) {
			return plan->trajectory_.joint_trajectory.points.back().time_from_start + transition_delay;
		}
		return ros::Duration(0);
	}
	arm_action(arm_action::Ptr previous) : parent(previous) { //assumed this has state ownership
		if (previous == nullptr) {
			vacuum_enabled = false;
		}
		else {
			vacuum_enabled = previous->vacuum_enabled;
		}
	}
	arm_action(arm_action::Ptr previous,tf::Pose end = tf::Pose(identity,tf::Vector3(0,0,0)),char region_ = SIMPLE_REGION_NONE) : 
	parent(previous), trajectory_end(end), region(region_) {
		if (previous == nullptr) {
			vacuum_enabled = false;
		}
		else {
			vacuum_enabled = previous->vacuum_enabled;
		}
	}
	~arm_action() {
		if (plan != nullptr) {
			delete plan;
		}
	}
};

struct pipeline_data {
	ros::Time time; //this is the true time at the end of the last action currently in the execution pipeline
	arm_action::Ptr action;
	bool success;
	pipeline_data(ros::Time time_in, arm_action::Ptr action_in) : time(time_in), action(action_in) {}
	pipeline_data() {
		time = ros::Time::now();
		action = nullptr;
	}
};


//utility methods

tf::Pose transform_from_vector(tf::Vector3 offset) {
	tf::Pose pose_out;
	pose_out.setRotation(identity);
	pose_out.setOrigin(offset);
	return pose_out;
}

bool within(tf::Vector3 test,tf::Vector3 a,tf::Vector3 b) {
	bool ret = ((test.getX() < std::max(a.getX(),b.getX())) && (test.getX() > std::max(a.getX(),b.getX())));
	ret = ret && ((test.getY() < std::max(a.getY(),b.getY())) && (test.getY() > std::max(a.getY(),b.getY())));
	ret = ret && ((test.getZ() < std::max(a.getZ(),b.getZ())) && (test.getZ() > std::max(a.getZ(),b.getZ())));
	return ret;
}

#endif
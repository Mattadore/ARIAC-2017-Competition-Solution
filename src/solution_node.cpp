//Author: Matt Buckley




//pls don't steal

//include files
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
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/LogicalCameraImage.h>
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

//core constants
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

//TODO: add whatever time moveit takes to figure itself out
const ros::Duration transition_delay(0.0);


const double intermediate_configuration_data[2][7] {
{-0.8, (3.0/2.0)*M_PI, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268},
{0.35, 1.5755329961346092, -1.3423701458943502, 1.685736119751326, 4.3690230176147775, -1.5707963257517727, 0.004736669665898268}};

const double true_center = -0.15;
//macros


#define SQUARE(x) x*x

//helpful constants
//acceptable radius squared

moveit_msgs::Constraints constrain;
const double AGV_acceptable_range_sq_ = SQUARE(AGV_acceptable_range_);

//find *REAL* joint tolerances and use those when doing move executions?
//formulate this into a scheduling problem?
//given a desired end state and blah blah
//online scheduling optimizer?
//plan generator with weights

//Notes about implementation details:

//There is a single list of all orders, a list of all kits, means to reference
//names back to kits and kits back to orders, and a second list of all kits copied from the first
//This second list is used to keep track of what orders have been completed. (tally list)
//When kits are assigned to an AGV, they are removed from the tally list.
//The ownership of the kit copy is then transferred to the appropriate AGV.

//Problem simplification assumptions:

//The first kit received must be completed as quickly as possible?
//It's more important to focus on individual kits because the speed gain of 
//keeping the agvs in constant circulation dwarfs the speed gain of efficient depositing

//TODO: optimize filtering
//TODO:
//read in orders
//process into kits
//have logical loop to go through every kit part and pick/place it
	//foreach kit part
	//find one of that type
	
	//pick it up
		//do pick logic
	//place it 

//TODO: update planner and controller on update() call to competition manager?


//enums (MANY ARE NOT USED)
enum process_states {
	ARM_nullptr, //so that default != 0 for reasons
	DEPOSIT,
	WAIT,
	COLLECT,
	COMPLETED,
	EXCEPTION
	//RETURN
};

enum AGV_states {
	AGV_nullptr,
	AGV_DELIVERING,
	AGV_DELIVERED,
	AGV_PREPARING_TO_DELIVER, //really short state
	AGV_READY_TO_DELIVER, //default, when the agv is *here*
	RETURNING
};

enum arm_regions {
	REGION_nullptr,
	AGV_1,
	AGV_2,
	BIN,
	GROUND,
	INTERMEDIATE_AGV_1,
	INTERMEDIATE_AGV_2,
	INTERMEDIATE_CONVEYOR,
	INTERMEDIATE_BIN,
	PREGRAB_CONVEYOR,
	PREGRAB_BIN,
	AGV_1_PREDROP,
	AGV_2_PREDROP,
	CONVEYOR,
	ARM_REGION_NUM
};

enum status_states {
	BOOLEAN_nullptr,
	BOOL_FALSE,
	BOOL_TRUE,
	ERROR_STATE
};

enum plan_types {
	PLAN_nullptr,
	PULLUP_PLAN,
	FORWARD_PLAN,
	GRAB_PLAN,
	RETRACT_PLAN,
	DROP_PLAN,
	EXCEPT_PLAN, //plan for when same name but different target, used in weird cases
	NO_PLAN,
	DEBUG_PLAN,
	PLANNING
};

enum competition_states {
	COMPETITION_nullptr,
	COMPETITION_END_GAME,
	COMPETITION_GO,
	COMPETITION_DONE,
	COMPETITION_INIT,
	COMPETITION_READY
};

enum state_types {
	STATE_nullptr,
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

enum intermediate_name {
	NEGATIVE_INTERMEDIATE,
	POSITIVE_INTERMEDIATE
};


/*
void createRegionPose(tf::Pose  in, arm_regions region, tf::Pose  out) {
	switch (region) {
	case AGV_1:
		break;
	case AGV_2:
		break;
	case AGV_1_PREDROP:
		break;
	case AGV_2_PREDROP:
		break;
	case BIN:
		break;
	case INTERMEDIATE_BIN: //start location
		break;
	case INTERMEDIATE_CONVEYOR:
		break;
	case INTERMEDIATE_AGV_2:
		break;
	case INTERMEDIATE_AGV_1:
		break;
	case CONVEYOR:
		break;
	}
}*/

//data structs

//TODO: add AGV status

moveit_msgs::Constraints make_constraint(tf::Pose end_location) {
	moveit_msgs::Constraints to_return;
	moveit_msgs::JointConstraint jointc;
	jointc.joint_name = "linear_arm_actuator_joint";
	jointc.position = end_location.getOrigin().y();
	jointc.tolerance_above = 0.05;
	jointc.tolerance_below = 0.05;
	jointc.weight = 1.0;
	to_return.joint_constraints.push_back(jointc);
	return to_return;
}

struct AGV_data {
	std::string num;
	std::string AGV_base_frame;
	std::string AGV_topic;
	AGV_data(char num_, std::string AGV_base_frame_, std::string AGV_topic_) :
	num({num_,'\0'}), AGV_base_frame(AGV_base_frame_),
	AGV_topic(AGV_topic_) {}
	AGV_data() {} //so I don't have to put this at the start of main construction
};

struct AGV_metadata {
	//bool unassigned;
	osrf_gear::Kit * current_kit;
	AGV_metadata() : current_kit(nullptr) {}
	bool assigned() {
		return current_kit == nullptr;
	}
};

//TODO: make "end_delay" a function pointer to something that decides what to do next?
struct arm_action {
	using Ptr = std::shared_ptr<arm_action>; 
	char planning_status = PIPELINE_NONE;
	char execution_status = PIPELINE_NONE;
	//unsigned char action_type;
	moveit::planning_interface::MoveGroup::Plan * plan = nullptr;
	tf::Pose trajectory_end;
	moveit::core::RobotState * start_state = nullptr;
	double end_delay = 0.0;
	bool true_pose = false;
	bool vacuum_enabled_end = false;
	bool pick_part = false;
	bool use_intermediate = false;
	arm_action::Ptr parent = nullptr;
	moveit_msgs::Constraints * constraints = nullptr;
	//maybe add what the state is here
	arm_action(arm_action::Ptr previous) : parent(previous) { //assumed this has state ownership
		if (previous == nullptr) {
			vacuum_enabled_end = false;
		}
		else {
			vacuum_enabled_end = previous->vacuum_enabled_end;
		}
	}
	arm_action(tf::Pose end, arm_action::Ptr previous) : parent(previous), trajectory_end(end) {
		if (previous == nullptr) {
			vacuum_enabled_end = false;
		}
		else {
			vacuum_enabled_end = previous->vacuum_enabled_end;
		}
	}
	~arm_action() {
		if (plan != nullptr) {
			delete plan;
		}
		if (start_state != nullptr) {
			delete start_state;
		}
	}
};

struct pipeline_data {
	ros::Time time; //this is the true time at the end of the last action currently in the execution pipeline
	arm_action::Ptr action;

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

// tf::Pose generate_intermediate(tf::Pose  to) { //creates a reasonable intermediate pose
// 	//tf::Pose * lesser = (abs(a.getOrigin().y())<abs(b.getOrigin().y()))?a:b; //take the pointer of the lesser
// 	//double y_val = (abs(lesser->getOrigin().y())<intermediate_limit) ? lesser->getOrigin().y() : intermediate_limit*(2*(lesser->getOrigin()>0)-1); //tries to use closer val
// 	//override this behavior for something a little cleaner
// 	double lesser = (intermediate_limit < abs(to.getOrigin().y()))?intermediate_limit:abs(to.getOrigin().y());
// 	double y_val = lesser*(2*(to.getOrigin().y()>0)-1); //uses edge val
// 	tf::Pose intermediate(identity,retraction_offset + tf::Vector3(0,y_val,0));
// 	//tf::Vector3 new_origin = retraction_offset + tf::Vector3(0,y_val,0);
// 	//new_origin.setY(y_val);
// 	//intermediate.setOrigin(new_origin);
// 	//intermediatePose.setRotation(intermediatePose.getRotation() * rotationOffset);
// 	return intermediate;
// 	ROS_INFO("FINISHED GENERATING INTERMEDIATE");
// }

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
		//subscribe("/ariac/laser_profiler_1", 10, laser_profiler_callback);
		subscribe("/ariac/gripper/state", 1, CompetitionInterface::vacuum_gripper_callback);
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

	//---------------callbacks (should not invoke any actions or action methods)
	static void competition_callback(const std_msgs::String::ConstPtr  msg) {
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

	static void current_score_callback(const std_msgs::Float32::ConstPtr  msg) {
		if (msg->data != current_score)
		{
			ROS_INFO_STREAM("Current score: " << msg->data);
		}
		current_score = msg->data;
	}

	static void vacuum_gripper_callback(const osrf_gear::VacuumGripperState::ConstPtr  msg) {
		state[GRIPPER_ATTACHED] = (msg->attached) ? BOOL_TRUE : BOOL_FALSE;
		state[GRIPPER_ENABLED] = (msg->enabled) ? BOOL_TRUE : BOOL_FALSE;
	}

	static void key_down_callback(const keyboard::Key::ConstPtr  msg) {
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

	static void order_callback(const osrf_gear::Order::ConstPtr  order_msg) {
		ROS_INFO_STREAM("Received order:\n" << *order_msg);
		//recheck_assignments = true; //check if AGV tray assignments have changed
		received_orders.push_back(*order_msg);
		for (int i = 0; i<order_msg->kits.size(); ++i) {
			if (kit_reference.count(order_msg->kits[i].kit_type)) {
				ROS_ERROR("MULTIPLE OF SAME KIT NAME");
				//totally able to happen maybe, but we want to know if it does
				//TODO: solution, make it a multi map? keep a copy for each?
			}
			kit_reference[order_msg->kits[i].kit_type] = &(received_orders[received_orders.size()-1].kits[i]); //references kit strings to kits
			kit_parent[order_msg->kits[i].kit_type] = &(received_orders[received_orders.size()-1]); //points to kit parent order
		}
	}

	static void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr  image_msg) {

	}

	//AGV_info[AGV_num-1].current_kit->kit_type

	//command methods
	static void send_AGV(char AGV_num,std::string kit_type) {
		ros::ServiceClient agv_client = nodeptr->serviceClient<osrf_gear::AGVControl>(AGV_info[AGV_num].AGV_topic.c_str());
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

	static void set_conveyor(float percent) {
		ros::ServiceClient conveyor_client = nodeptr->serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
		osrf_gear::ConveyorBeltControl srv;
		srv.request.state.power = percent;
		conveyor_client.call(srv);
		if (!srv.response.success) {
			ROS_ERROR("Unable to command conveyor");
		}
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
			return get_state(AGV_1_STATE);
		}
		else if (agv_id == 2) {
			return get_state(AGV_2_STATE);
		}
		else {
			ROS_ERROR("Invalid AGV");
			return 0;
		}
	}

	static const std::vector<osrf_gear::Order>  get_orders() {
		return received_orders;
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

protected:
	static std::vector<osrf_gear::Order> received_orders; //just an order list
	static std::map<std::string, osrf_gear::Kit*> kit_reference; //a way to look up kits
	static std::map<std::string, osrf_gear::Order*> kit_parent; //a way to look up orders from kits

	static std::map<std::string,ros::Subscriber> subscriptions;
	static double current_score;
	static sensor_msgs::JointState current_joint_state;
	static unsigned char state[NUM_STATES]; //holds state machine info
	static AGV_data AGV_info[2]; //basic agv information
	// static unsigned char state_diff[NUM_STATES]; //holds diff info
	// static unsigned char state_old[NUM_STATES]; //holds old state machine info
	// static unsigned char state_if_diff[NUM_STATES]; //holds old state machine info
	static ros::NodeHandle * nodeptr;
	CompetitionInterface() {};
};

//Planner class
class Planner {
public:
	Planner() : arm_control_group("manipulator"), plan_thread(boost::bind(&Planner::parallel_plan,this)) {
		arm_control_group.setPoseReferenceFrame("/world");
		arm_control_group.setWorkspace(-20000,-20000,-20000,20000,20000,20000);
		generic_state = new moveit::core::RobotState(*(arm_control_group.getCurrentState()));
		jointmsg_sample = *ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/ariac/arm/state");
		for (int i=0;i<7;++i) { 
			intermediate_points[0].positions.push_back(intermediate_configuration_data[0][i]);
			intermediate_points[1].positions.push_back(intermediate_configuration_data[1][i]);
			intermediate_points[0].velocities.push_back(0);
			intermediate_points[1].velocities.push_back(0);
			// std_msgs::Float64 f;
			// f.data = intermediate_configuration_data[0][i];
			// intermediate_configuration_vectors[0].push_back(f);
			// f.data = intermediate_configuration_data[1][i];
			// intermediate_configuration_vectors[1].push_back(f);
		}
		intermediate_points[0].time_from_start = ros::Duration(2.0);
		intermediate_points[1].time_from_start = ros::Duration(2.0);
	}

	void wait_for_plan_complete() {
		boost::unique_lock<boost::mutex> current_plan_lock(current_plan_mutex);
		while (currently_planning != nullptr) {
			current_plan_condition.wait(current_plan_lock);			
		}
	}


	//TODO: send condition notificiation when planning queue is cleared
	//TODO we can do this with a flag to indicate that the thread is blocking on an arm action in the planner
	void wait_until_planned(arm_action::Ptr to_wait) {
		boost::unique_lock<boost::mutex> current_plan_lock(current_plan_mutex);
		while ((to_wait->planning_status != PIPELINE_COMPLETE) && (to_wait->planning_status != PIPELINE_NONE)) {
			current_plan_condition.wait(current_plan_lock); //wait until *something* has finished planning
		}
	}

	void add_actions(std::vector<arm_action::Ptr> * to_plan) {
		{ //scope kills lock appropriately
			boost::unique_lock<boost::mutex> plan_lock(plan_mutex);
			for (int i=0;i<to_plan->size();++i) {
				if ((*to_plan)[i]->planning_status != PIPELINE_PROCESSING) {
					(*to_plan)[i]->planning_status = PIPELINE_WAITING;
				}
				plan_queue.push_back((*to_plan)[i]);
			}
		}
		plan_condition.notify_all(); //do this while mutex still locked to ensure notified threads are waiting when lock deleted
	}

	void add_action(arm_action::Ptr to_plan) {
		{
			boost::unique_lock<boost::mutex> plan_lock(plan_mutex);
			plan_queue.push_back(to_plan);
			if (to_plan->planning_status != PIPELINE_PROCESSING) {
				to_plan->planning_status = PIPELINE_WAITING;
			}
  		}
		plan_condition.notify_all();
	}

	void remove_action(arm_action::Ptr to_remove) {
		{
			boost::unique_lock<boost::mutex> plan_lock(plan_mutex); //plays safe with lock conditions on planning list
			//LONGTERM TODO test if needs to be removed?
			for (std::list<arm_action::Ptr>::iterator i = plan_queue.begin(); i != plan_queue.end(); ++i) {
				if 
					(*i == to_remove) {
					(*i)=nullptr; //remove self from planning queue
				}
			}
		}
		if (to_remove == currently_planning) {
			boost::unique_lock<boost::mutex> plan_lock(current_plan_mutex); //to ensure not deleted WHILE planning
			//pretty sure this is all I need becauase it should block until planning completes
			//gets kicked out at the end of planning anyway
		}
	}

	void clear() {
		boost::unique_lock<boost::mutex> plan_lock(plan_mutex);
		for (std::list<arm_action::Ptr>::iterator i = plan_queue.begin(); i != plan_queue.end(); ++i) {
			(*i)->planning_status = PIPELINE_NONE;
		}
		plan_queue.clear();
	}
	void set_rotation_offset(tf::Quaternion offset) {
		rotation_offset = offset;
	}

protected:


	void plan_intermediate(arm_action::Ptr to_plan) {
		ROS_INFO("INTERMEDIATE PLAN STARTING!");

		to_plan->plan = new moveit::planning_interface::MoveGroup::Plan();
		if (to_plan->trajectory_end.getOrigin().getY() < 0) {
			to_plan->plan->trajectory_.joint_trajectory.points.push_back(intermediate_points[NEGATIVE_INTERMEDIATE]);
		}
		else {
			to_plan->plan->trajectory_.joint_trajectory.points.push_back(intermediate_points[POSITIVE_INTERMEDIATE]);
		}
		to_plan->plan->trajectory_.joint_trajectory.joint_names = jointmsg_sample.joint_names;

		ROS_INFO("INTERMEDIATE PLAN COMPLETE!");
	}

	void plan(arm_action::Ptr to_plan) {
		ROS_INFO("PLAN STARTING!");
		to_plan->plan = new moveit::planning_interface::MoveGroup::Plan();
		moveit_msgs::RobotTrajectory traj_msg;
		std::vector<geometry_msgs::Pose> pose_list;

		tf::Pose to_add = to_plan->trajectory_end;
		if (!to_plan->true_pose) {
			to_add.setRotation(to_add.getRotation() * rotation_offset);
		}
		geometry_msgs::Pose geometry_pose;
		tf::poseTFToMsg(to_add,geometry_pose);
		ROS_INFO("PLAN_B");

		pose_list.push_back(geometry_pose);

		//set plan state to parent traj last state
		//moveit::core::RobotState start_state;
		//TODO: separate start states for intermediate and primary action
		//TODO: make this nicer? find a better way to combine two actions without screwing around in the controller
		//TODO: directly generate an intermediate plan in code separately from the other stuff?
		if ((to_plan->parent != nullptr) && (to_plan->start_state == nullptr)) {
			const trajectory_msgs::JointTrajectory plan_trajectory = to_plan->parent->plan->trajectory_.joint_trajectory;
			size_t pose_number = plan_trajectory.points.size() - 1;
			to_plan->start_state = generic_state;
			ROS_INFO("PLAN_B2");
			moveit::core::jointTrajPointToRobotState(plan_trajectory, pose_number, *(to_plan->start_state));
		}
		ROS_INFO("PLAN_A");
		if (to_plan->parent == nullptr) {
			arm_control_group.setStartStateToCurrentState();
		}
		else {
			arm_control_group.setStartState(*(to_plan->start_state));
		}
		//arm_control_group.setStartStateToCurrentState();

		//compute trajectory
		double fraction = arm_control_group.computeCartesianPath(pose_list, 0.05, 0.0, traj_msg, true);
		if (to_plan->end_delay > 0) {
			auto last_point = traj_msg.joint_trajectory.points.back();
			for (int i=0;i<last_point.velocities.size();++i) {
				last_point.velocities[i] = 0;
				last_point.accelerations[i] = 0;
			}
			traj_msg.joint_trajectory.points.push_back(last_point);
			traj_msg.joint_trajectory.points.back().time_from_start += ros::Duration(std::min(to_plan->end_delay/10.0,0.1));
			traj_msg.joint_trajectory.points.push_back(last_point);
			traj_msg.joint_trajectory.points.back().time_from_start += ros::Duration(to_plan->end_delay);
		}

		// do time parameterization
		// robot_trajectory::RobotTrajectory traj(to_plan->start_state->getRobotModel(), "manipulator");
		// traj.setRobotTrajectoryMsg(*(to_plan->start_state), traj_msg);
		// trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
		// bool success = parameterizer.computeTimeStamps(traj);
		// traj.getRobotTrajectoryMsg(traj_msg);
		

		//assign
		//to_plan->start_state = start_state;
		to_plan->plan->trajectory_ = traj_msg;
		ROS_INFO("PLAN COMPLETE!");
	}

	//---------------planner logic
	void parallel_plan() {
		ros::Duration wait(0.01);
		while (true) {
			{
				wait.sleep();
				ROS_INFO_THROTTLE(1,"Parallel control running");
				currently_planning = nullptr;
				boost::unique_lock<boost::mutex> current_plan_lock(current_plan_mutex);
				{ //mutex lock scope
					boost::unique_lock<boost::mutex> plan_lock(plan_mutex);
					while (plan_queue.size() == 0) { //todo: other conditions if necessary
						plan_condition.wait(plan_lock); //apparently this unlocks until awoken
					}
					currently_planning = plan_queue.front();
					plan_queue.pop_front();
					//don't notify here, only notify when adding
				} //unlock

				currently_planning->planning_status = PIPELINE_PROCESSING;
				if ((currently_planning->plan != nullptr) && (currently_planning->parent == nullptr))  { //don't do this if it has a parent?
					ROS_INFO("Plan already exists?");
					delete currently_planning->plan;
					currently_planning->plan = nullptr;
				}
				if (currently_planning->plan == nullptr) {
					if (currently_planning->use_intermediate) {
						plan_intermediate(currently_planning);
					}
					else {
						plan(currently_planning); //finally do planning		
					}
					//currently_planning->plan->trajectory_.joint_trajectory.points[0].time_from_start = ros::Duration(0.001); //why not
				}
				else {
					ROS_INFO("PLAN SKIPPING");
				}
				currently_planning->planning_status = PIPELINE_COMPLETE;
			}
			current_plan_condition.notify_all();
		}
	}
	trajectory_msgs::JointTrajectoryPoint intermediate_points[2];
	moveit::core::RobotState * generic_state;
	moveit::planning_interface::MoveGroup arm_control_group;
	boost::condition_variable plan_condition;
	boost::condition_variable current_plan_condition; 
	boost::mutex plan_mutex;
	boost::mutex current_plan_mutex;
	arm_action::Ptr currently_planning;
	boost::thread plan_thread;
	std::list<arm_action::Ptr> plan_queue;
	control_msgs::JointTrajectoryControllerState jointmsg_sample;

	tf::Quaternion rotation_offset;
};



//Controller class


class Controller {
public:
	Controller(Planner * planner_) : arm_control_group("manipulator"), control_thread(boost::bind(&Controller::parallel_control,this)),
	planner(planner_) {
		arm_control_group.setPoseReferenceFrame("/world");
		arm_control_group.setWorkspace(-20000,-20000,-20000,20000,20000,20000);
		std::vector<std_msgs::Float64> intermediate_configuration_vectors[2];
		//doing some prior assignment
		// intermediate_points[0].positions = decltype(intermediate_points[0].positions)(intermediate_configuration_vectors[0].begin(),intermediate_configuration_vectors[0].end());
		// intermediate_points[1].positions = decltype(intermediate_points[1].positions)(intermediate_configuration_vectors[1].begin(),intermediate_configuration_vectors[1].end());
	}

	void wait_for_move_complete() {
		boost::unique_lock<boost::mutex> current_control_lock(current_control_mutex);
		while (currently_executing != nullptr) { //probably unnecessary
			current_control_condition.wait(current_control_lock);
		}
		//current_control_condition->wait(current_control_lock);
	}
	//TODO: this will *break* if the action is deleted. fix
	void wait_until_executed(arm_action::Ptr to_wait) {
		boost::unique_lock<boost::mutex> current_control_lock(current_control_mutex);
		while ((to_wait->execution_status != PIPELINE_COMPLETE) && (to_wait->execution_status != PIPELINE_NONE)) {
			current_control_condition.wait(current_control_lock);
			ROS_INFO("Waiting; queried order status is: %d",to_wait->execution_status);
		}
	}

	void add_actions(std::vector<arm_action::Ptr> * to_move) {
		{ //scope kills lock appropriately
			boost::unique_lock<boost::mutex> control_lock(control_mutex);
			for (int i=0;i<to_move->size();++i) {
				if ((*to_move)[i]->execution_status != PIPELINE_PROCESSING) {
					(*to_move)[i]->execution_status = PIPELINE_WAITING;
				}
				control_queue.push_back((*to_move)[i]);
			}
		}
		control_condition.notify_all();
	}

	void add_action(arm_action::Ptr to_move) {
		{
			boost::unique_lock<boost::mutex> control_lock(control_mutex);
			control_queue.push_back(to_move);
			if (to_move->execution_status != PIPELINE_PROCESSING) { //this doesn't matter *that* much, so long as it isn't PIPELINE_NONE or completed
				to_move->execution_status = PIPELINE_WAITING;
			}
	  	}
		control_condition.notify_all();
	}

	void remove_action(arm_action::Ptr to_remove) {
		{
			boost::unique_lock<boost::mutex> control_lock(control_mutex); //plays safe with lock conditions on execution list
			//LONGTERM TODO test if needs to be removed?
			for (std::list<arm_action::Ptr>::iterator i = control_queue.begin(); i != control_queue.end(); ++i) {
				if (*i == to_remove) {
					*i=nullptr; //remove self from execution queue
				}
			}
		}
		if (to_remove == currently_executing) {
			stop_controller();
			boost::unique_lock<boost::mutex> control_lock(current_control_mutex); //to ensure not deleted WHILE planning

			//pretty sure this is all I need becauase it should block until planning completes
			//gets kicked out at the end of planning anyway
		}
	}
	void clear() {
		boost::unique_lock<boost::mutex> control_lock(control_mutex); //plays safe with lock conditions on planning list
		for (std::list<arm_action::Ptr>::iterator i = control_queue.begin(); i != control_queue.end(); ++i) {
			(*i)->execution_status = PIPELINE_NONE;
		}
		control_queue.clear();
	}
	void stop_controller() {
		//TODO: implement
	}
protected:
	void grab_break() { //hardly thread-safe
		ROS_INFO("STARTING GRAB BREAK THREAD");
		arm_action::Ptr start_action = currently_executing;
		ros::Rate looprate(50);
		while (start_action == currently_executing) {
			if (CompetitionInterface::get_state(GRIPPER_ATTACHED) == BOOL_TRUE) {
				ROS_INFO_THROTTLE(1,"GRAB BREAK!");
				arm_control_group.stop();
			}
			looprate.sleep();
		} 
		ROS_INFO("GRAB BREAK THREAD TERMINATED");
	}

	void move(arm_action::Ptr to_move) {
		ROS_INFO("MOVE STARTING!");
		//moveit::core::robotStateToRobotStateMsg (*(arm_control_group.getCurrentState()), to_move->plan->start_state_); 
		//possibly remove any start state requirement more than it already is
		//arm_control_group.setStartStateToCurrentState();

		//arm_control_group.setStartStateToCurrentState();
		boost::thread * break_thread = nullptr;
		if (to_move->pick_part) {
			break_thread = new boost::thread(boost::bind(&Controller::grab_break,this));
		}
		arm_control_group.execute(*(to_move->plan));
		//ros::Duration(to_move->end_delay).sleep(); //todo: do this cleaner
		CompetitionInterface::toggle_vacuum(to_move->vacuum_enabled_end);
		ROS_INFO("MOVE COMPLETE!");
		if (break_thread != nullptr) {
			delete break_thread;
		}
	}

	//---------------controller logic
	void parallel_control() {
		ros::Duration wait(0.01);
		while (true) {
			{
				wait.sleep();
				ROS_INFO_THROTTLE(1,"Parallel control running");
				boost::unique_lock<boost::mutex> current_control_lock(current_control_mutex);
				currently_executing = nullptr;
				{ //mutex lock scope
					boost::unique_lock<boost::mutex> control_lock(control_mutex); //wait for lock on control queue
					while (control_queue.size() == 0) { //todo: other conditions if necessary
						control_condition.wait(control_lock); //apparently this unlocks until awoken
					}
					currently_executing = control_queue.front();
					control_queue.pop_front();
					currently_executing->execution_status = PIPELINE_PROCESSING;
				} //unlock
				if (currently_executing->plan == nullptr) { //ensure there is a plan to execute
					planner->wait_until_planned(currently_executing);
				}
				move(currently_executing); //execute (blocking) move call

				//boost::thread block_move_thread(boost::bind(Controller::parallel_control,this));
				//block_move_thread.
				//while ()


				//reset system, prepare to lose lock
				currently_executing->execution_status = PIPELINE_COMPLETE;

				//currently_executing = nullptr;
			}
			current_control_condition.notify_all();
		}
	}
 

	//for letting action execution wait
	moveit::planning_interface::MoveGroup arm_control_group;
	boost::condition_variable control_condition;
	boost::condition_variable current_control_condition;
	boost::mutex control_mutex;
	boost::mutex current_control_mutex;
	arm_action::Ptr currently_executing;
	boost::thread control_thread;
	Planner * planner;
	std::list<arm_action::Ptr> control_queue; 
};

//Competition manager class

class CompetitionManager {
public:

	//for testing
	void make_moving(trajectory_msgs::JointTrajectory  a) {
		//double pos_at_start = a.points[0].positions[1];
		for (int i=1;i<a.points.size();++i) {
			a.points[i].velocities[1] -= 0.18;
			a.points[i].positions[1] -= a.points[i].time_from_start.toSec()*0.18;
		}
	}
	//TODO: make sure this is finished
	std::string get_closest_part(std::string part_name) {
		//TODO: do not go for parts that are out of play
		std::string best = "";
		double shortest = 0; //keep track of shortest distance squared
		char frame_name[50];
		tf::StampedTransform delta_trans,arm_trans;
		listener.lookupTransform("world","vacuum_gripper_link",ros::Time(0),arm_trans);
		for (int i = 0;;++i) { //just get the nearest part for now
			snprintf(frame_name,50,"%s_%d_frame",part_name.c_str(),i);
			//THIS ASSUMES WE CAN SEE ALL PARTS
			if (!(frame_names.count(std::string(frame_name)))) break;
			if (listener.frameExists(frame_name)) {
				listener.lookupTransform("world",frame_name,ros::Time(0),delta_trans);
				double curr_length = (delta_trans.getOrigin()-arm_trans.getOrigin()).length2();
				if ((curr_length < shortest) || (best == "")) {
					best = std::string(frame_name);
					shortest = curr_length;
				}
			}
		}
		return best;
	}


	// std::list<unsigned char> search(unsigned char start_state, unsigned char end_state) { //bfs search for state path
	// 	std::map<unsigned char state,unsigned char parent> stored;
	// 	std::list<unsigned char> queue,result;
	// 	queue.push_back(start_state);
	// 	visited[start_state] = REGION_nullptr;
	// 	if (start_state == end_state) {
	// 		result.push_back(start_state);
	// 		return result;
	// 	}
	// 	while (queue.size() > 0) {
	// 		unsigned char top = *(queue.begin());
	// 		queue.pop_front();
	// 		for (std::list::<unsigned char>::iterator i = transitions[top]; i!=transitions[top].end(); ++i) {
	// 			if (*i == end_state) { //don't feel like breaking out of the loop
	// 				unsigned char step = *i;
	// 				while (step != REGION_nullptr) {
	// 					result.push_front(step);
	// 					step = stored[step]; //go to parent
	// 				}
	// 				return result;
	// 			}
	// 			else if (!stored.count(*i)) { //if not yet queued
	// 				stored[*i] = top;
	// 				queue.push_back(*i);
	// 			}
	// 		}
	// 	}
	// 	return result;
	// }


	//TODO: add this
	
	// void AGV1_callback(const std_msgs::String::ConstPtr  msg) {dd
	// 	std::string state = msg->data;
	// 	if (state == "")
	// }



	//actions and planning
	void update() { //happens once per spin cycle
		const std::vector<osrf_gear::Order>  orders = CompetitionInterface::get_orders();
		if (orders.size() > number_orders) {
			for (int order_i = (number_orders-1); order_i<orders.size(); ++order_i) {
				for (int kit_i = 0; kit_i<orders[order_i].kits.size(); ++kit_i) {
					kit_tally[orders[order_i].kits[kit_i].kit_type] = orders[order_i].kits[kit_i]; //makes new second copy of all kits
				}
			}
			number_orders = orders.size();
		}
		if (CompetitionInterface::get_state(COMPETITION_STATE) == COMPETITION_DONE) {
			terminated = true;
		}
		//check all toggled states
		//look for any funny business
	}

 // //TODO: FINISH
	// std::string generate_pickup_plan(std::string target,std::string plan_parent = "",std::string name = "") { //integrate pick plan into execution pipe
	// 	vector<arm_action::Ptr > * pickup_plan = new vector<arm_action::Ptr > ();
	// 	arm_action::Ptr temp;
	// 	tf::StampedTransform target_transform;
	// 	if (name == "") {
	// 		name = target+"_pickup";
	// 	}
	// 	listener.lookupTransform("world","target",ros::Time(0),target_transform);
	// 	pickup_plan->insert(temp);

	// 	return name;
	// }

	// void execute_pickup_plan() { 

	// }

	// void generate_dropoff_plan() { //integrate pick plan into execution pipe

	// }

	// void execute_dropoff_plan() { 

	// }

	//integrate b into a, maybe have switches happen at given times
	void integrate(arm_action::Ptr a, arm_action::Ptr b) {
		ros::Duration a_end_time = a->plan->trajectory_.joint_trajectory.points.back().time_from_start;
		for (int i=1;i<b->plan->trajectory_.joint_trajectory.points.size();++i) {
			b->plan->trajectory_.joint_trajectory.points[i].time_from_start += a_end_time;
			a->plan->trajectory_.joint_trajectory.points.push_back(b->plan->trajectory_.joint_trajectory.points[i]);
		}
	}

	void combine_actions(std::vector<arm_action::Ptr> * action_list) {
		
	}

	//testing
	void grab(std::string action_name,std::string part_name) {
		tf::Pose grab_pose = ObjectTracker::get_grab_pose(part_name);
		motion_paths[action_name] = new std::vector<arm_action::Ptr>();
	 	motion_paths[action_name]->push_back(arm_action::Ptr(new arm_action(grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2)),nullptr)));
	 	motion_paths[action_name]->back()->vacuum_enabled_end = true;
	 	motion_paths[action_name]->push_back(arm_action::Ptr(new arm_action(grab_pose,motion_paths[action_name]->back())));
	 	//motion_paths[action_name]->push_back(new arm_action(grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2)),motion_paths[action_name]->back()));
	 	planner.add_actions(motion_paths[action_name]);
	 	planner.wait_until_planned(motion_paths[action_name]->back());
	 	//integrate(motion_paths[action_name]->front(),motion_paths[action_name]->back());
	 	//make_moving(motion_paths[action_name]->front()->plan->trajectory_.joint_trajectory);
	 	//motion_paths[action_name]->pop_back();
	 	controller.add_actions(motion_paths[action_name]);
	}

	// void drop(std::string action_name,std::string drop_frame,tf::Pose grab_offset,geometry_msgs::Pose tray_offset_geom) {
	// 	tf::Pose tray_offset;
	// 	tf::poseMsgToTF(tray_offset_geom,tray_offset);
	// 	arm_action::Ptr parent_action = nullptr;
	// 	tf::StampedTransform temp,arm_location;
	// 	tf::Pose offset,drop_offset,intermediate,intermediate2;
	// 	listener.lookupTransform("world",drop_frame,ros::Time(0),temp);
	// 	listener.lookupTransform("world","vacuum_gripper_link",ros::Time(0),arm_location);
	// 	offset = transform_from_vector(tf::Vector3(0,0,0.2));
	// 	drop_offset = temp*tray_offset*grab_offset*offset;
	// 	intermediate = generate_intermediate(temp);
	// 	intermediate2 = transform_from_vector(tf::Vector3(0,0.9*(2*(intermediate.getOrigin().y()>0)-1),0))*intermediate;

	// 	//offset = transform_from_vector(tf::Vector3(0,0,0.01));
	// 	motion_paths[action_name] = new std::vector<arm_action::Ptr>();
	// 	motion_paths[action_name]->push_back(new arm_action(intermediate,parent_action));
	// 	(*(motion_paths[action_name]))[0]->vacuum_enabled_end = true;
	// 	//(*(motion_paths[action_name]))[0]->lock_arm = true;
	// 	motion_paths[action_name]->push_back(new arm_action(intermediate2,(*(motion_paths[action_name]))[0]));
	// 	motion_paths[action_name]->push_back(new arm_action(drop_offset,(*(motion_paths[action_name]))[1]));
	// 	(*(motion_paths[action_name]))[2]->vacuum_enabled_end = false;
	// 	//(*(motion_paths[action_name]))[2]->true_pose = true;
	// 	(*(motion_paths[action_name]))[2]->end_delay = 0.8;
	// 	motion_paths[action_name]->push_back(new arm_action(intermediate,(*(motion_paths[action_name]))[2]));
	// 	//(*(motion_paths[action_name]))[2]->lock_arm = true;
	// 	//(*(motion_paths[action_name]))[1]->end_delay = 1.0;
	// 	planner.add_actions(motion_paths[action_name]);
	// 	controller.add_actions(motion_paths[action_name]);
	// }


	// void minidrop(std::string action_name,std::string drop_frame,tf::Pose drop_pose) {
	// 	arm_action::Ptr parent_action = nullptr;
	// 	tf::StampedTransform temp,arm_location;
	// 	tf::Pose offset,drop_offset;
	// 	listener.lookupTransform("world",drop_frame,ros::Time(0),temp);
	// 	listener.lookupTransform("world","vacuum_gripper_link",ros::Time(0),arm_location);
	// 	drop_offset = transform_from_vector(temp.getOrigin())*drop_pose;

	// 	//offset = transform_from_vector(tf::Vector3(0,0,0.01));
	// 	motion_paths[action_name] = new std::vector<arm_action::Ptr>();
	// 	tf::Vector3 new_origin = arm_location.getOrigin();
	// 	new_origin.setZ(drop_offset.getOrigin().getZ());
	// 	tf::Pose temp2 = transform_from_vector(new_origin);
	// 	motion_paths[action_name]->push_back(new arm_action(temp2,parent_action));
	// 	(*(motion_paths[action_name]))[0]->vacuum_enabled_end = true;
	// 	motion_paths[action_name]->push_back(new arm_action(drop_offset,(*(motion_paths[action_name]))[0]));
	// 	(*(motion_paths[action_name]))[1]->end_delay = 1.5;
	// 	(*(motion_paths[action_name]))[1]->vacuum_enabled_end = false;

	// 	planner.add_actions(motion_paths[action_name]);
	// 	controller.add_actions(motion_paths[action_name]);
	// }

	// void drop(std::string drop_frame,std::string action_name,tf::Pose grab_offset) {
	// 	tf::Pose tray_offset = transform_from_vector(tf::Vector3(0,0,0));
	// 	geometry_msgs::Pose tray_offset_geom;
	// 	tf::poseTFToMsg(tray_offset,tray_offset_geom);
	// 	drop(drop_frame,action_name,grab_offset,tray_offset_geom);
	// }

	#define blocking_call(type,name,...) { \
	 	type(name,__VA_ARGS__); \
	 	ROS_INFO("Starting %s",name); \
	 	controller.wait_until_executed(motion_paths[name]->back()); \
	}


	//TODO: write method to do a kit here, part by part
	//if during the completion of a part, we find out that there is a new priority kit
	//go work on that once current part is finished placing
	void execute_kit_simple() {

	}

	//make motion along belt have v = 0.2 ish
	void homogenize_for_belt(trajectory_msgs::JointTrajectory  a,double distance,double velocity = 0.2,double distance_per = 0.05) {
		//double pos_at_start = a.points[0].positions[1];
		trajectory_msgs::JointTrajectory b(a);
		double point_num = a.points.size()-1;
		double time_per = (distance/velocity)/point_num;
		double total_time = a.points.back().time_from_start.toSec();
		ROS_INFO("0 member name: %s",a.joint_names[0].c_str());
		for (int i=1;i<a.points.size();++i) {
			double current_velocity = distance_per/(b.points[i].time_from_start-b.points[i-1].time_from_start).toSec();
			double scale = velocity/current_velocity;
			for (int j=0;j<a.points[i].velocities.size();++j) {
				a.points[i-1].velocities[j]*=scale;
				a.points[i].time_from_start *= scale;
				if (!a.points[i].accelerations.empty()) {
					//TODO: should this be i-1?
					a.points[i-1].accelerations[j]*=scale*scale; //I think? sort of just guessing at this one
				}
			}
			//a.points[i].velocities[0] = -velocity; //probably will help
			//a.points[i].accelerations[0] = 0; //probably will help
			//a.points[i].accelerations.clear(); //probably will help?
			a.points[i].time_from_start = ros::Duration(i*time_per);
		}
		a.points.pop_back();
		ROS_INFO_STREAM(a);
	}
	
	//if no pipeline data passed, will just act as if called on current time
	pipeline_data simple_grab_moving(std::string part_name,pipeline_data data_in = pipeline_data()) {
		if (data_in.action != nullptr) {
			data_in.action->vacuum_enabled_end = true;
		}
		else {
			CompetitionInterface::toggle_vacuum(true);
		}
		tf::Pose current_grab = ObjectTracker::get_grab_pose(part_name,data_in.time+ros::Duration(0.05));
		arm_action::Ptr dummy_action(new arm_action(current_grab,data_in.action));
		dummy_action->vacuum_enabled_end = true;
		dummy_action->pick_part = true;
		for (char i=0;i<8;++i) {
			dummy_action->trajectory_end = current_grab;
			if (i>0) {
				delete dummy_action->plan;
				dummy_action->plan = nullptr;
				dummy_action->planning_status = PIPELINE_NONE;
			}
			planner.add_action(dummy_action);
			planner.wait_until_planned(dummy_action);
			ROS_INFO("ITER COMPLETE");
			ros::Time end_time = data_in.time + dummy_action->plan->trajectory_.joint_trajectory.points.back().time_from_start;
			ROS_INFO("duration is %f",dummy_action->plan->trajectory_.joint_trajectory.points.back().time_from_start.toSec());
			current_grab = ObjectTracker::get_grab_pose(part_name,end_time + ros::Duration(0.05));

		}
		double dist = 1.0;
		tf::Pose transform_pose = tf::Pose(identity,tf::Vector3(0,-dist,0)) * current_grab;
		arm_action::Ptr slide_action(new arm_action(transform_pose,dummy_action));
		slide_action->vacuum_enabled_end = true;
		planner.add_action(slide_action);
		planner.wait_until_planned(slide_action);
		homogenize_for_belt(slide_action->plan->trajectory_.joint_trajectory,dist);
		integrate(dummy_action,slide_action);


		controller.add_action(dummy_action);
		//controller.add_action(slide_action);
		controller.wait_until_executed(dummy_action);
		tf::StampedTransform arm_location;
		listener.lookupTransform("world","vacuum_gripper_link",ros::Time(0),arm_location);
		tf::Pose next_location = tf::Pose(identity,tf::Vector3(0,0,0.2)+arm_location.getOrigin());
		arm_action::Ptr retract_action(new arm_action(next_location,nullptr));
		retract_action->vacuum_enabled_end = true;


		planner.add_action(retract_action);
		planner.wait_until_planned(retract_action);
		//Movement pipeline is recreated here
		ros::Time pipeline_time = ros::Time::now()+transition_delay+retract_action->plan->trajectory_.joint_trajectory.points.back().time_from_start;
		controller.add_action(retract_action);
		//controller.wait_until_executed(retract_action);
		//delete dummy_action;
		//delete slide_action;
		return pipeline_data(pipeline_time,retract_action);
	}

	//---------------control logic
	void arm_process() {
		ROS_INFO("Logic process started");
		ros::Duration(1).sleep();
		ros::Duration wait_rate(0.03);
		/*while (received_orders.size() == 0) {
			wait_rate.sleep();
		}*/

		CompetitionInterface::toggle_vacuum(false);
		ros::Duration(2.5).sleep();

		// tf::Pose arm_trans;
		// arm_trans = ObjectTracker::get_tray_pose(1);
		// //arm_trans=tf::Pose(identity,tf::Vector3(0,0,0.25))*arm_trans;

		// arm_action::Ptr intermediate_action(new arm_action(arm_trans,nullptr));
		// intermediate_action->use_intermediate = true;
		// intermediate_action->trajectory_end = arm_trans;
		// arm_action::Ptr align_action(new arm_action(arm_trans,intermediate_action));
		// //align_action.pick_part = true;
		// align_action->vacuum_enabled_end = true;
		// align_action->end_delay = 3.0;
		// align_action->trajectory_end = arm_trans;
		// tf::Pose lift_pose = tf::Pose(identity,tf::Vector3(0,0,1.0))*arm_trans;
		// arm_action::Ptr lift_action(new arm_action(lift_pose,align_action));
		// lift_action->vacuum_enabled_end = true;
		// planner.add_action(intermediate_action);
		// planner.add_action(align_action);
		// planner.add_action(lift_action);
		// controller.add_action(intermediate_action);
		// controller.add_action(align_action);
		// controller.wait_until_executed(align_action);
		// auto last = lift_action->plan->trajectory_.joint_trajectory.points.back();
		// lift_action->plan->trajectory_.joint_trajectory.points.clear();
		// lift_action->plan->trajectory_.joint_trajectory.points.push_back(last);
		// ros::Duration(3.0).sleep();
		// controller.add_action(lift_action);
		// controller.wait_until_executed(lift_action);
		//delete align_action;

		CompetitionInterface::start_competition();


		// trajectory_msgs::JointTrajectory traj;
		// control_msgs::JointTrajectoryControllerState jointmsg;
		// ROS_INFO("Rotating");
		// if (ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/ariac/arm/state") == nullptr) {
		// 	ROS_INFO("carp");
		// }
		// jointmsg  = *ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/ariac/arm/state");
		// jointmsg.desired.positions[1]-=M_PI;
		// jointmsg.desired.positions[0]+=0.7;
		// jointmsg.desired.time_from_start = ros::Duration(0.1);
		// traj.points.push_back(jointmsg.desired);
		// traj.joint_names = jointmsg.joint_names;
		// joint_pub.publish(traj);




		// CompetitionInterface::toggle_vacuum(true);
		// tf::Pose grab_pose = ObjectTracker::get_grab_pose("pulley_part_2");
		// arm_action grab_action(grab_pose,nullptr);
		// grab_action.pick_part = true;
		// grab_action.vacuum_enabled_end = true;
		// grab_action.end_delay = 8.0;
		// tf::Pose pull_pose = grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2));
		// arm_action pull_action(pull_pose,grab_action);
		// pull_action.pick_part = false;
		// pull_action.vacuum_enabled_end = true;
		// pull_action.end_delay = 0.0;
		// planner.add_action(grab_action);
		// planner.add_action(pull_action);
		// planner.wait_until_planned(pull_action);
		// ROS_INFO_STREAM("TRAJ" << grab_action.plan->trajectory_.joint_trajectory); 
		// controller.add_action(grab_action);
		// controller.add_action(pull_action);
		// controller.wait_until_executed(pull_action);

		// CompetitionInterface::toggle_vacuum(false);




		// CompetitionInterface::toggle_vacuum(true);
		// tf::Pose grab_pose = ObjectTracker::get_grab_pose("pulley_part_3");
		// arm_action grab_action(grab_pose,nullptr);
		// grab_action.parent = nullptr;
		// grab_action.start_state = nullptr;
		// grab_action.trajectory_end = grab_pose;
		// grab_action.pick_part = false;
		// grab_action.vacuum_enabled_end = true;
		// grab_action.end_delay = 0.0;
		// arm_action wait_action(grab_pose,grab_action);
		// wait_action.parent = grab_action;
		// wait_action.start_state = nullptr;
		// wait_action.trajectory_end = grab_pose;
		// wait_action.pick_part = false;
		// wait_action.vacuum_enabled_end = true;
		// wait_action.end_delay = 3.0;
		// tf::Pose pull_pose = grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2));
		// arm_action pull_action(pull_pose,grab_action);
		// pull_action.parent = grab_action;
		// pull_action.start_state = nullptr;
		// pull_action.trajectory_end = pull_pose;
		// pull_action.pick_part = false;
		// pull_action.vacuum_enabled_end = true;
		// pull_action.end_delay = 0.0;
		// planner.add_action(grab_action);
		// planner.add_action(wait_action);
		// planner.add_action(pull_action);
		// planner.wait_until_planned(pull_action);
		// ROS_INFO_STREAM("TRAJ" << wait_action.plan->trajectory_.joint_trajectory); 
		// controller.add_action(grab_action);
		// controller.add_action(wait_action);
		// controller.add_action(pull_action);
		// controller.wait_until_executed(pull_action);

		//TODO: if velocity mag < some value, do a thing
		// CompetitionInterface::start_competition();

		// ros::Rate part_wait_rate(30);
		// while (!ObjectTracker::part_exists("piston_rod_part_clone_0")) {
		// 	part_wait_rate.sleep();
		// }
		// ros::Duration(13.0).sleep();
		// simple_grab_moving("piston_rod_part_clone_0");


		//blocking_call(grab, "piston_rod_part_3", "pulley_part_2")

		//do stuff

		//CompetitionInterface::end_competition();

		//for each kit 
		//for (std::map<std::string,osrf_gear::Kit>::iterator i = kit_tally.begin();++i;i!=kit_tally.end()) {
			//execute a method to do a kit

		//}

		//***
		//have option for interrupts
		//***


		//wait on order queue
		//ASSIGN THE AGV NEAR CONVEYOR START TO FIRST ORDER, SECOND TO SECOND

		//assign a kit to an AGV (assign multiple?)
			//pop kit from tally index
			//set AGV pointer to kit
			//keep track of added parts in agv
			//only if there are no parts on the tray can you change the agv task.
			//only if all the tray parts correspond to the kit parts, send the kit.



	}

	//got bored, this is a weirdly hacky solution. avoided an even more hacky one
	//template<typename array>
	//void link(array array_in) { //keeping this all pre-c++11
		//unsigned char array_size = sizeof(array)/sizeof(unsigned char);
/*
	#define region_link(...) link_f(__VA_ARGS__, REGION_nullptr)
	void link_f(...) {
	    va_list list;
	    va_start(list, argument_num);
	    std::vector<unsigned char> locations;
	    unsigned char current_location = va_arg(list, unsigned char);
	    while (current_location != REGION_nullptr) {
	    	for (unsigned char j = 0; j < locations.size(); ++j) {
	    		transitions[locations[j]].push_back(current_location);
	    		transitions[current_location].push_back(locations[j]);
	    	}
	    	locations.push_back();
	    	current_location = va_arg(list, unsigned char);
		}
	    va_end(list);
	}


	//initializes transition vector
	void set_transitions() {
		region_link(INTERMEDIATE_AGV_1,INTERMEDIATE_AGV_2,INTERMEDIATE_CONVEYOR,INTERMEDIATE_BIN);
		region_link(AGV_1_DROP,AGV_1_PREDROP);
		region_link(AGV_1_PREDROP,AGV_1,INTERMEDIATE_AGV_1);


	}



	arm_regions locate(tf::Vector3 position) {
		if (position.getZ() < somenumber) {
			return GROUND; //hopefully never
		}
		else if (abs(position.getX()) > somenumber) {
			if (position.getZ() < somenumber) {
				//we're picking up a part

			}
			//it's in the region zone thing
		}
		else if (position.getX() < somenumber) {
			return CONVEYOR;
		}
		else {
			if (position.getY() > somenumber) {
				if (position.getZ() > somenumber) {
					return AGV_1_PREDROP;
				}
				else {
					return AGV_1; //just guessing which agv is which
				}
			}
			else if (position.getY() < somenumber) {
				if (position.getZ() > somenumber) {
					return AGV_2_PREDROP;
				}
				else {
					return AGV_2; //just guessing which agv is which
				}
			else {
				return INTERMEDIATE;
			}
		}
	}
*/


	//CompetitionManager() : buffer(ros::Duration(1.0)),
	//	listener(buffer), server(buffer, "tf_action", false) {
	CompetitionManager(ros::NodeHandle * nodeptr_) : listener(ros::Duration(1.0)), controller(&planner), 
	logic_thread(boost::bind(&CompetitionManager::arm_process,this)) {
		nodeptr = nodeptr_;
		joint_pub = nodeptr->advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 20);

		terminated = false;
		number_orders = 0;
	  	//ensure buffer is completely initialized?

		//std::vector<std::string> frame_names_ = tf; //add all frame names to set
		//for (int i=0;i<frame_names_.size();++i) {
		//	frame_names.insert(frame_names_[i]);
		//}


		tf::Quaternion offset;
		offset.setEuler(M_PI/2,-M_PI/2,0);
		planner.set_rotation_offset(offset);
	}

	~CompetitionManager () {
		//clear memory
		for (std::map<std::string,std::vector<arm_action::Ptr>*>::iterator i = motion_paths.begin();i!=motion_paths.end();++i) {
			delete (*i).second;
		}
	}

	//external logic
	bool ok() {
		return !terminated;
	}

	Planner planner;
	Controller controller;

protected:

	//ros::Publisher joint_trajectory_publisher;
	unsigned char number_orders;
	bool terminated; //execution end flag
	//std::map<std::string, unsigned char> object_type_counts; //a way to check how many of each thing there are
	//std::set<std::string> successfully_used_parts;
	//std::set<std::string> dropped_parts;

	//keeps track of how we think kit completion is going
	//this part metadata helps us track how we're completing the kits, and where parts go and why
	//can be reconfigured if necessary
	std::map<std::string, osrf_gear::Kit> kit_tally; //remove elements as able, maybe add an order version?
	AGV_metadata AGV_info[2];

	std::map<std::string, std::vector<arm_action::Ptr>*> motion_paths;
	tf::TransformListener listener; //TODO: make this part of competition interface/special tf tracker (part indexer interface)

	//Something like this is needed for looking up tf names and stuff
	std::set<std::string> frame_names;

	boost::thread logic_thread;
	ros::NodeHandle * nodeptr; //not particularly clean but the node needs to die if this goes out of scope anyway

	bool part_waiting;
	std::string waiting_for;
	ros::Publisher joint_pub;

	//std::list transitions[ARM_REGION_NUM];

};

// THIS OBJECT FILE OWNS THE DEFINITIONS FOR COMPETITIONINTERFACE
std::vector<osrf_gear::Order> CompetitionInterface::received_orders; //just an order list
std::map<std::string, osrf_gear::Kit*> CompetitionInterface::kit_reference; //a way to look up kits
std::map<std::string, osrf_gear::Order*> CompetitionInterface::kit_parent; //a way to look up orders from kits

std::map<std::string,ros::Subscriber> CompetitionInterface::subscriptions;
double CompetitionInterface::current_score;
sensor_msgs::JointState CompetitionInterface::current_joint_state;
unsigned char CompetitionInterface::state[NUM_STATES]; //holds state machine info
AGV_data CompetitionInterface::AGV_info[2]; //basic agv information
// unsigned char state_diff[NUM_STATES]; //holds diff info
// unsigned char state_old[NUM_STATES]; //holds old state machine info
// unsigned char state_if_diff[NUM_STATES]; //holds old state machine info
ros::NodeHandle * CompetitionInterface::nodeptr;


int main(int argc, char ** argv) {
	ros::init(argc, argv, "solution_node");
	ros::NodeHandle node;
	CompetitionManager manager(&node);
	tf::TransformListener listener;

	ROS_INFO("STARTING\n");


	//constrain.joint_constraints.push_back(jointc);

	CompetitionInterface::initialize_interface(&node);
	ObjectTracker::initialize_tracker(&node,&listener);
	ros::Time tf_publish = ros::Time::now();
	ros::Duration tf_frequency(0.1);
	ros::Rate spinRate(100);
	while (ros::ok() && manager.ok()) { //functionally equivalent to spin() but more customizable
		spinRate.sleep();
		ros::spinOnce();
		manager.update();
		if ((tf_publish+tf_frequency)>ros::Time::now()) {
			ObjectTracker::publish_tfs();
			tf_publish = ros::Time::now();
		}
	}
	return 0;
}
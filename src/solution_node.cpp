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
#include <include/competition_interface.h>
#include <include/utility.h>
#include <include/planner.h>
#include <include/controller.h>

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


//Compiling pressing todos

//TODO: faulty part handling
//TODO: dropped part detection and handling
//TODO: find set joint configs for part drop on both sides 
//(away from agv movement) and a dynamic one for belt grabbing; write these in
//TODO: write method to clear planning queue, use it if I get a faulty part or smth
//TODO: in these cases, replan; or, I guess, just always block during part dropping
//      I have to anyway in case I use camera and do pose correction
//TODO: generate and run test cases with weird configs
//TODO: add more metadata to object info
//TODO: let objects know if they can be correctly done?
//TODO: !!!!!!!only do objects with unknown perfect configuration last
//TODO: poll bin service, log responses into struct

//TODO: make grid part grabber grab inside most parts that are not central ones
//!!!TODO: grab where there is the most overlap, based on part radius, gripper radius, grid structure
//TODO: faulty part to gleam information


//TODO: have something check the bin before submission; if I can fix something, fix something.
//TODO: this is necessary because new information may not come until end.
//TODO: submit if no new information can be gleamed?
//TODO: feel around for dropped part? don't feel where existing parts are
//TODO: maybe don't submit until both kits are done?
//TODO: maybe use second agv as storage?
//TODO: look at finishing discrete state-space implementation
//!!!!!TODO: rank these ideas as best scoring/most feasible to least?
//TODO: worst case: don't worry about orientation


//TODO: resolve internal handoff in object tracker for how that is handled, how grabbing a part works when I can't see it, all that fun stuff
//TODO: have some space for keeping track of all these unknowns
//TODO: re-write competition manager to have logic tracking routines
//TODO: direct or inferrential logic tracker? - probably direct, easier and more powerful; inferrential is safer and more robust
//TODO: probably also responsible for grid management? or is that the object tracker..
//TODO: do this within competitionmanager, it'll be way faster

//TODO: list of competing goals and important it is I think I can accomplish them.
//TODO: create heuristics for that
//TODO: grid detector
//TODO: finish simple actions
//TODO: split up into (at least) multiple header files

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

//TODO: make virtual pickup objects and update their numbering
//TODO: it must always be one of the grids? edge case??
//TODO: mark tf with *blahblah*_expected and update when I find out the truth

//TODO: maybe region detection

//TODO: make a lot less laggy
	//averaging
//TODO: make grid manager

//TODO: calculate drop time on conveyor using maths
//TODO: find conveyor top surface height

//TODO: if belt part spacing is good, use belt, otherwise wait it out

//!!!NOTE: possibly take all parts from conveyor and put on an agv.
//worst case, I spend some time fishing around for parts.
//we could also stack them. stack af. especially since I know their true center.
//we would also know how high they are stacked.
//could also pre-configure stacking positions for each part


//changelog:
//found the pointclouds for all parts (not included)
//started adding structs for bin tracking and grid alignment detection
//revised a bunch of code, worked to standardize action methods
//added subscription methods to most competition stuff I wasnt' listening to before
//revised how joint value moves are done
//added method to estimate time a given joint value move will take

//Competition manager class

class CompetitionManager {
public:

	//for testing
	void make_moving(trajectory_msgs::JointTrajectory & a) {
		//double pos_at_start = a.points[0].positions[1];
		for (int i=1;i<a.points.size();++i) {
			a.points[i].velocities[1] -= 0.18;
			a.points[i].positions[1] -= a.points[i].time_from_start.toSec()*0.18;
		}
	}

	//TODO: write competition tracker

	//TODO: re-write, move to object tracker
	// std::string get_closest_part(std::string part_name) {
	// 	//TODO: do not go for parts that are out of play?
	// 	std::string best = "";
	// 	double shortest = 0; //keep track of shortest distance squared
	// 	char frame_name[50];
	// 	tf::StampedTransform delta_trans,arm_trans;
	// 	listener.lookupTransform("world","vacuum_gripper_link",ros::Time(0),arm_trans);
	// 	for (int i = 0;;++i) { //just get the nearest part for now
	// 		snprintf(frame_name,50,"%s_%d_frame",part_name.c_str(),i);
	// 		//THIS ASSUMES I CAN SEE ALL PARTS
	// 		if (!(frame_names.count(std::string(frame_name)))) break;
	// 		if (listener.frameExists(frame_name)) {
	// 			listener.lookupTransform("world",frame_name,ros::Time(0),delta_trans);
	// 			double curr_length = (delta_trans.getOrigin()-arm_trans.getOrigin()).length2();
	// 			if ((curr_length < shortest) || (best == "")) {
	// 				best = std::string(frame_name);
	// 				shortest = curr_length;
	// 			}
	// 		}
	// 	}
	// 	return best;
	// }


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



	//actions and planning
	void update() { //happens once per spin cycle
		std::vector<osrf_gear::Order> & orders = CompetitionInterface::get_orders();
		if (orders.size() > number_orders) {
			for (int order_i = (number_orders-1); order_i<orders.size(); ++order_i) {
				for (int kit_i = 0; kit_i<orders[order_i].kits.size(); ++kit_i) {
					kit_tally[&(orders[order_i].kits[kit_i])] = kit_metadata(orders[order_i].kits[kit_i]); //makes new second copy of all kits
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


	//TODO: write method to do a kit here, part by part
	//if during the completion of a part, I find out that there is a new priority kit
	//go work on that once current part is finished placing
	void execute_kit_simple() {

	}

	//make motion along belt have v = 0.2 ish
	void homogenize_for_belt(trajectory_msgs::JointTrajectory & a,double distance,double velocity = 0.2,double distance_per = 0.05) {
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
	
	//testingd
	
	pipeline_data simple_grab(std::string part_name,pipeline_data data_in = pipeline_data()) {
		tf::Pose grab_pose = ObjectTracker::get_grab_pose(part_name);
		arm_action::Ptr align_action(new arm_action(data_in.action,grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2)),REGION_BINS));
	 	arm_action::Ptr grab_action(new arm_action(align_action,grab_pose,REGION_BIN_GRAB));
	 	grab_action->vacuum_enabled = true;
	 	grab_action->pick_part = true;
	 	grab_action->end_delay = 2.0;
	 	arm_action::Ptr lift_action(new arm_action(grab_action,grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2)),REGION_BIN_GRAB));
	 	planner.add_action(align_action);
	 	planner.add_action(grab_action);
	 	planner.add_action(lift_action);
	 	controller.add_action(align_action);
	 	controller.add_action(grab_action);
	 	controller.add_action(lift_action);
	 	controller.wait_until_executed(grab_action);
	 	pipeline_data return_data(ros::Time::now()+lift_action->plan->trajectory_.joint_trajectory.points.back().time_from_start,lift_action);
	 	return return_data;
	}

	//if no pipeline data passed, will just act as if called on current time
	pipeline_data simple_grab_moving(std::string part_name,pipeline_data data_in = pipeline_data()) {
		tf::Pose current_grab = ObjectTracker::get_grab_pose(part_name,data_in.time+ros::Duration(0.05));
		arm_action::Ptr dummy_action(new arm_action(data_in.action,current_grab,REGION_BINS));
		dummy_action->vacuum_enabled = true;
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
		arm_action::Ptr slide_action(new arm_action(dummy_action,transform_pose,REGION_BINS));
		planner.add_action(slide_action);
		planner.wait_until_planned(slide_action);
		homogenize_for_belt(slide_action->plan->trajectory_.joint_trajectory,dist);
		integrate(dummy_action,slide_action);


		controller.add_action(dummy_action);
		//controller.add_action(slide_action);
		controller.wait_until_executed(dummy_action);
		tf::StampedTransform arm_location = ObjectTracker::get_gripper_pose();
		tf::Pose next_location = tf::Pose(identity,tf::Vector3(0,0,0.2)+arm_location.getOrigin());
		arm_action::Ptr retract_action(new arm_action(nullptr,next_location,REGION_BINS));
		retract_action->vacuum_enabled = true;

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

	//NOTE: this drop does not correct for issues with incorrect pose grip, assumes pose correctly handled
	pipeline_data simple_drop(char agv_number,tf::Pose drop_offset = tf::Pose(identity,tf::Vector3(0,0,0)),pipeline_data data_in = pipeline_data()) {
		tf::Pose agv_pose = ObjectTracker::get_tray_pose(agv_number);
		arm_action::Ptr move_to_intermediate(new arm_action(data_in.action,agv_pose,REGION_BINS));
		move_to_intermediate->use_intermediate = true;
		move_to_intermediate->vacuum_enabled = true;
		char agv_region = (agv_number == 1) ? REGION_AGV1 : REGION_AGV2;
		arm_action::Ptr move_to_tray(new arm_action(move_to_intermediate,tf::Pose(identity,tf::Vector3(0,0,0.3))*agv_pose*drop_offset,agv_region));
	 	planner.add_action(move_to_intermediate);
	 	planner.add_action(move_to_tray);
	 	controller.add_action(move_to_intermediate);
	 	controller.add_action(move_to_tray);
	 	std::vector<arm_action::Ptr> standard_actions; //TODO: do something similar for waving under camera
	 	std::vector<arm_action::Ptr> trash_actions; //TODO: do something similar for waving under camera
	 	arm_action::Ptr retract_action;
	 	//controller.wait_until_executed(move_to_tray);
	 	osrf_gear::LogicalCameraImage quality_sensor_reading = CompetitionInterface::get_quality_control_msg(agv_number);

	 	//me trying out a forked plan

 		trash_actions.push_back(arm_action::Ptr(new arm_action(move_to_tray,agv_pose,agv_region)));
 		trash_actions.back()->use_trash = true;
 		trash_actions.back()->end_delay = 0.5;
 		trash_actions.push_back(arm_action::Ptr(new arm_action(trash_actions.back(),move_to_tray->trajectory_end,agv_region)));
 		trash_actions.back()->vacuum_enabled = false;
 		trash_actions.push_back(arm_action::Ptr(new arm_action(trash_actions.back(),agv_pose,agv_region)));
 		trash_actions.back()->use_intermediate = true;
 		planner.add_actions(&trash_actions);

 		//if nothing bad happens
	 	standard_actions.push_back(arm_action::Ptr(new arm_action(move_to_tray,move_to_tray->trajectory_end,agv_region)));
	 	standard_actions.back()->end_delay = 0.5;
	 	standard_actions.push_back(arm_action::Ptr(new arm_action(move_to_tray,agv_pose,agv_region)));
	 	standard_actions.back()->vacuum_enabled = false;
	 	standard_actions.back()->use_intermediate = true;
 		planner.add_actions(&standard_actions);

 		planner.wait_until_planned(standard_actions.back());
 		controller.wait_until_executed(move_to_tray);
 
		ros::Duration pipeline_time;
		std::vector<arm_action::Ptr>* action_list;
	 	if (!quality_sensor_reading.models.empty()) { //Oh boy
	 		//NOTE: this is where I will possibly work backwards to bin alignment
	 		//NOTE: only assumes one possible model; under assumption that we will always throw away our faulty models
	 		//NOTE: a lot of this stuff could be better done state-based, but that doesn't mesh well with forward planning.
	 		//NOTE: find some way to integrate state stuff and forward planning together well?
	 		//!!!!!!!TODO: plan multiple possible outcomes?
	 		//basically just scoots the arm over a bit before dropping so it falls
	 		//off the side, instead of in the intended drop
	 		action_list = &standard_actions;
	 	}
	 	else {
	 		action_list = &standard_actions;
	 	}
 		controller.add_actions(action_list);
 		for (arm_action::Ptr action : (*action_list)) {
	 		pipeline_time += action->plan->trajectory_.joint_trajectory.points.back().time_from_start;
 		}
	 	return pipeline_data(ros::Time::now()+pipeline_time,action_list->back());
	}

	//---------------object space management logic

	//greedy kit assignment
	void assign_kits() {
		if (AGV_info[0].assigned() && AGV_info[1].assigned()) {
			return;
		}
		for (std::map<osrf_gear::Kit *,kit_metadata>::iterator tally_i = kit_tally.begin();tally_i!=kit_tally.end();++tally_i) {
			if (tally_i->second.agv_number == 0) {
				if (!(AGV_info[0].assigned())) {
					assign_kit(tally_i->first, 1);
				}
				else if (!(AGV_info[1].assigned())) {
					assign_kit(tally_i->first, 2);
				}
			}
		}
	}


	//---------------control logic
	void arm_process() {
		ROS_INFO("Logic process started");
		ros::Duration(1).sleep();

		ros::Duration wait_rate(0.03);
		CompetitionInterface::toggle_vacuum(false);
		CompetitionInterface::start_competition();

		while (CompetitionInterface::get_state(COMPETITION_STATE) == COMPETITION_GO) {
			wait_rate.sleep();
			if (kit_tally.empty()) {
				continue;
			}

			assign_kits();




		}




		//ros::Duration(2.5).sleep();

		// tf::Pose agv1_pose = ObjectTracker::get_tray_pose(1);
		// arm_action::Ptr intermediate_move(new arm_action(nullptr,agv1_pose,REGION_BINS));
		// intermediate_move->use_intermediate = true;
		// ros::Time start_time = ros::Time::now();
		// planner.add_action(intermediate_move);
		// controller.add_action(intermediate_move);
		// controller.wait_until_executed(intermediate_move);
		// ros::Duration time_taken = ros::Time::now() - start_time;
		// ros::Duration expected_time = intermediate_move->plan->trajectory_.joint_trajectory.points.back().time_from_start;
		// ROS_INFO("Time expected: %fs, time taken: %fs",expected_time.toSec(),time_taken.toSec());




		//CompetitionInterface::start_competition();


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
		// grab_action.vacuum_enabled = true;
		// grab_action.end_delay = 8.0;
		// tf::Pose pull_pose = grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2));
		// arm_action pull_action(pull_pose,grab_action);
		// pull_action.pick_part = false;
		// pull_action.vacuum_enabled = true;
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
		// grab_action.vacuum_enabled = true;
		// grab_action.end_delay = 0.0;
		// arm_action wait_action(grab_pose,grab_action);
		// wait_action.parent = grab_action;
		// wait_action.start_state = nullptr;
		// wait_action.trajectory_end = grab_pose;
		// wait_action.pick_part = false;
		// wait_action.vacuum_enabled = true;
		// wait_action.end_delay = 3.0;
		// tf::Pose pull_pose = grab_pose*tf::Pose(identity,tf::Vector3(0,0,0.2));
		// arm_action pull_action(pull_pose,grab_action);
		// pull_action.parent = grab_action;
		// pull_action.start_state = nullptr;
		// pull_action.trajectory_end = pull_pose;
		// pull_action.pick_part = false;
		// pull_action.vacuum_enabled = true;
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

	// #define region_link(...) link_f(__VA_ARGS__, REGION_nullptr)
	// void link_f(...) {
	//     va_list list;
	//     va_start(list, argument_num);
	//     std::vector<unsigned char> locations;
	//     unsigned char current_location = va_arg(list, unsigned char);
	//     while (current_location != REGION_nullptr) {
	//     	for (unsigned char j = 0; j < locations.size(); ++j) {
	//     		transitions[locations[j]].push_back(current_location);
	//     		transitions[current_location].push_back(locations[j]);
	//     	}
	//     	locations.push_back();
	//     	current_location = va_arg(list, unsigned char);
	// 	}
	//     va_end(list);
	// }


	// //initializes transition vector
	// void set_transitions() {
	// 	region_link(INTERMEDIATE_AGV_1,INTERMEDIATE_AGV_2,INTERMEDIATE_CONVEYOR,INTERMEDIATE_BIN);
	// 	region_link(AGV_1_DROP,AGV_1_PREDROP);
	// 	region_link(AGV_1_PREDROP,AGV_1,INTERMEDIATE_AGV_1);


	// }





	//CompetitionManager() : buffer(ros::Duration(1.0)),
	CompetitionManager(ros::NodeHandle * nodeptr_) : controller(&planner), 
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
	void assign_kit(osrf_gear::Kit * to_assign,char agv_number) {
		if (kit_tally.count(to_assign) == 0) {
			ROS_ERROR("Kit does not exist");
			return;
		}
		if ((agv_number!=1)&&(agv_number!=2)) {
			ROS_ERROR("AGV does not exist");
			return;
		}
		kit_metadata & data = kit_tally[to_assign];
		AGV_metadata & agv = AGV_info[agv_number-1];
		if (agv.current_kit == to_assign) {
			return;
		}
		if (agv.current_kit != nullptr) { //if currently assigned
			if (kit_tally.count(agv.current_kit) != 0) { //if kit exists
				kit_tally[agv.current_kit].agv_number = 0; //reassign kit to nothing
			}
		}
		data.agv_number = agv_number;
		agv.current_kit = to_assign;
	}

	void complete_kit(osrf_gear::Kit * to_assign) {
		if (kit_tally.count(to_assign) == 0) {
			ROS_ERROR("Kit does not exist");
			return;
		}
		kit_metadata & data = kit_tally[to_assign];
		if ((data.agv_number!=1)&&(data.agv_number!=2)) {
			ROS_ERROR("AGV does not exist");
			return;
		}
		AGV_metadata & agv = AGV_info[data.agv_number-1];
		
		

		// data.agv_number = agv_number; // Check if this true
		agv.current_kit = to_assign;
	}

	struct AGV_metadata {
		//bool unassigned;
		osrf_gear::Kit * current_kit = nullptr;
		bool assigned() {
			return current_kit == nullptr;
		}
	};

	struct kit_metadata {
		char agv_number = 0;
		osrf_gear::Kit kit_clone;
		osrf_gear::Kit * kit_pointer;
		bool assigned() {
			return (agv_number != 0);
		}
		kit_metadata(){}
		kit_metadata(osrf_gear::Kit & kit_in) : kit_clone(kit_in), kit_pointer(& kit_in) {}
	};

	//ros::Publisher joint_trajectory_publisher;
	unsigned char number_orders;
	bool terminated; //execution end flag
	//std::map<std::string, unsigned char> object_type_counts; //a way to check how many of each thing there are
	//std::set<std::string> successfully_used_parts;
	//std::set<std::string> dropped_parts;

	//keeps track of how I think kit completion is going
	//this part metadata helps us track how we're completing the kits, and where parts go and why
	//can be reconfigured if necessary
	std::map<osrf_gear::Kit*, kit_metadata> kit_tally; //maps real kits to copied objects
	AGV_metadata AGV_info[2];

	std::map<std::string, std::vector<arm_action::Ptr>*> motion_paths;

	//Something like this is needed for looking up tf names and stuff
	std::set<std::string> frame_names;

	boost::thread logic_thread;
	ros::NodeHandle * nodeptr; //not particularly clean but the node needs to die if this goes out of scope anyway

	bool part_waiting;
	std::string waiting_for;
	ros::Publisher joint_pub;

};


int main(int argc, char ** argv) {
	ros::init(argc, argv, "solution_node");
	ros::NodeHandle node;
	CompetitionManager manager(&node);
	tf::TransformListener listener;

	ROS_INFO("STARTING\n");

	CompetitionInterface::initialize_interface(&node);
	ObjectTracker::initialize_tracker(&node,&listener);
	ros::Time tf_publish = ros::Time::now();
	ros::Duration tf_frequency(0.1);
	ros::Rate spinRate(100);
	while (ros::ok() && manager.ok()) { //functionally equivalent to spin() but more customizable
		spinRate.sleep();
		ros::spinOnce();
		manager.update();
		// if ((tf_publish+tf_frequency)>ros::Time::now()) {
		// 	ObjectTracker::publish_tfs();
		// 	tf_publish = ros::Time::now();
		// }
	}
	return 0;
}
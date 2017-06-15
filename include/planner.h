#ifndef PLANNER_H_
#define PLANNER_H_

#include <include/utility.h>
#include <include/competition_interface.h>
#include <cmath>

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
			trashcan_points[0].positions.push_back(trashcan_configuration_data[0][i]);
			trashcan_points[1].positions.push_back(trashcan_configuration_data[1][i]);
			trashcan_points[0].velocities.push_back(0);
			trashcan_points[1].velocities.push_back(0);
			conveyor_point.positions.push_back(conveyor_configuration_data[i]);
			conveyor_point.velocities.push_back(0);
			scan_point.positions.push_back(scanning_configuration_data[i]);
			scan_point.velocities.push_back(0);
			// std_msgs::Float64 f;
			// f.data = intermediate_configuration_data[0][i];
			// intermediate_configuration_vectors[0].push_back(f);
			// f.data = intermediate_configuration_data[1][i];
			// intermediate_configuration_vectors[1].push_back(f);
		}
		// intermediate_points[0].time_from_start = ros::Duration(2.0);
		// intermediate_points[1].time_from_start = ros::Duration(2.0);
		// trashcan_points[0].time_from_start = ros::Duration(2.0);
		// trashcan_points[1].time_from_start = ros::Duration(2.0);
	}

	void wait_for_plan_complete() {
		boost::unique_lock<boost::mutex> current_plan_lock(current_plan_mutex);
		while (currently_planning != nullptr) {
			current_plan_condition.wait(current_plan_lock);			
		}
	}


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


	//TODO: not done, resolve
	void remove_action(arm_action::Ptr to_remove) {
		{
			boost::unique_lock<boost::mutex> plan_lock(plan_mutex); //plays safe with lock conditions on planning list
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
			(*i)->planning_status = PIPELINE_COMPLETE;
		}
		plan_queue.clear();
	}
	void set_rotation_offset(tf::Quaternion offset) {
		rotation_offset = offset;
	}

protected:

	ros::Duration get_action_time(trajectory_msgs::JointTrajectoryPoint & a,trajectory_msgs::JointTrajectoryPoint & b) {
		double arm_times[] {3,2.16,2.16,3.15,3.2,3.2,3.2}; //roughly
		//double accel_times = [3,2,2,3,3,3,3]; //again, roughly
		std::vector<double> times;
		for (int i=0;i<a.positions.size();++i) {
			double distance = std::abs(a.positions[i]-b.positions[i]);
			if (i>0) { //if an angle
				distance = M_PI-std::abs(distance - M_PI); //i believe this is correct
			}
			times.push_back((distance / arm_times[i])+0.1);
		}
		return ros::Duration(*std::max_element(times.begin(),times.end())); //seems legit
	}

	void plan_joints(arm_action::Ptr to_plan) {
		ROS_INFO("INTERMEDIATE PLAN STARTING!");

		to_plan->plan = new moveit::planning_interface::MoveGroup::Plan();

		if (to_plan->parent != nullptr) {
			if (to_plan->parent->planning_failure) {
				ROS_WARN("Parent had a planning failure, terminating planning");
				to_plan->planning_failure = true;
				return;
			}
		}

		if (to_plan->use_intermediate) {
			if (to_plan->trajectory_end.getOrigin().getY() < 0) {
				to_plan->plan->trajectory_.joint_trajectory.points.push_back(intermediate_points[NEGATIVE_CONFIGURATION]);
			}
			else {
				to_plan->plan->trajectory_.joint_trajectory.points.push_back(intermediate_points[POSITIVE_CONFIGURATION]);
			}
		}
		else if (to_plan->use_trash) {
			if (to_plan->trajectory_end.getOrigin().getY() < 0) {
				to_plan->plan->trajectory_.joint_trajectory.points.push_back(trashcan_points[NEGATIVE_CONFIGURATION]);
			}
			else {
				to_plan->plan->trajectory_.joint_trajectory.points.push_back(trashcan_points[POSITIVE_CONFIGURATION]);
			}
		}
		else if (to_plan->use_conveyor) {
			//limit is 2.1
			trajectory_msgs::JointTrajectoryPoint custom_conveyor(conveyor_point);
			double y_position = to_plan->trajectory_end.getOrigin().getY();
			y_position -= 0.1; //why not
			const double limit = 2.0;
			if (y_position > limit) {
				y_position = limit;
			}
			else if (y_position < -limit) {
				y_position = -limit;
			}
			custom_conveyor.positions[0] = y_position;
			to_plan->plan->trajectory_.joint_trajectory.points.push_back(custom_conveyor);
		}
		else if (to_plan->scan) {
			to_plan->plan->trajectory_.joint_trajectory.points.push_back(scan_point);
		}
		else {
			ROS_ERROR("No valid configuration set");
		}

		to_plan->plan->trajectory_.joint_trajectory.joint_names = jointmsg_sample.joint_names;
		if (to_plan->parent != nullptr) {
			ros::Duration move_time = get_action_time(to_plan->parent->plan->trajectory_.joint_trajectory.points.back(), to_plan->plan->trajectory_.joint_trajectory.points.back());
			to_plan->plan->trajectory_.joint_trajectory.points.back().time_from_start = move_time;
		}
		else {
			control_msgs::JointTrajectoryControllerState jointmsg = *ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/ariac/arm/state");
			ros::Duration move_time = get_action_time(jointmsg.actual, to_plan->plan->trajectory_.joint_trajectory.points.back());
			to_plan->plan->trajectory_.joint_trajectory.points.back().time_from_start = move_time;
		}
		if (to_plan->end_delay > 0) {
			ROS_INFO("Check after planning...");
			auto last_point = to_plan->plan->trajectory_.joint_trajectory.points.back();
			for (int i=0;i<last_point.velocities.size();++i) {
				ROS_INFO("Check 2 after planning...");
				last_point.velocities[i] = 0;
			}
			ROS_INFO("Check after 3 planning...");
			to_plan->plan->trajectory_.joint_trajectory.points.push_back(last_point);
			to_plan->plan->trajectory_.joint_trajectory.points.back().time_from_start += ros::Duration(std::min(to_plan->end_delay/10.0,0.1));
			to_plan->plan->trajectory_.joint_trajectory.points.push_back(last_point);
			to_plan->plan->trajectory_.joint_trajectory.points.back().time_from_start += ros::Duration(to_plan->end_delay);
		}
		ROS_INFO("INTERMEDIATE PLAN COMPLETE!");
	}

	void plan(arm_action::Ptr to_plan) {
		ROS_INFO("PLAN STARTING!");
		to_plan->plan = new moveit::planning_interface::MoveGroup::Plan();

		moveit_msgs::RobotTrajectory traj_msg;

		tf::Pose to_add = to_plan->trajectory_end;
		if (!to_plan->true_pose) {
			to_add.setRotation(to_add.getRotation() * rotation_offset);
		}
		ROS_INFO("PLAN_B");

		std::vector<geometry_msgs::Pose> pose_list;
		geometry_msgs::Pose geometry_pose;
		tf::poseTFToMsg(to_add,geometry_pose);
		pose_list.push_back(geometry_pose);


		//set plan state to parent traj last state
		//moveit::core::RobotState start_state;
		if (to_plan->parent != nullptr) {
			if (to_plan->parent->planning_failure) {
				ROS_WARN("Parent had a planning failure, terminating planning");
				to_plan->planning_failure = true;
				return;
			}
			const trajectory_msgs::JointTrajectory plan_trajectory = to_plan->parent->plan->trajectory_.joint_trajectory;
			size_t pose_number = plan_trajectory.points.size() - 1;
			ROS_INFO("PLAN_B1");
			moveit::core::jointTrajPointToRobotState(plan_trajectory, pose_number, *generic_state);
			arm_control_group.setStartState(*generic_state);
		}
		else {
			ROS_INFO("PLAN_B2");
			arm_control_group.setStartStateToCurrentState();
		}

		//compute trajectory
		double fraction = arm_control_group.computeCartesianPath(pose_list, 0.05, 0.0, traj_msg, true);
		if (to_plan->perturb) {
			int attempt_max = 8;
			for (int test_i=0;(test_i<attempt_max)&&(fraction < 0.97);++test_i) {
				ROS_WARN("PLANNING RETRY: ATTEMPT #%d",test_i+2);

				double angle = M_PI*2.0*((double)test_i)/((double)attempt_max);
				tf::Vector3 plan_offset = tf::Vector3(sin(angle)*0.02,cos(angle)*0.02,0);

				std::vector<geometry_msgs::Pose> pose_list_2;
				tf::Pose to_add_2 = to_add * tf::Transform(identity,plan_offset);
				tf::poseTFToMsg(to_add_2,geometry_pose);
				pose_list_2.push_back(geometry_pose);

				// to_plan->trajectory_end.getBasis().getRPY(r,p,y);
				tf::Vector3 pos = to_add_2.getOrigin();
				ROS_ERROR("Perturb xyz: %f %f %f", pos.x(),pos.y(),pos.z());

				fraction = arm_control_group.computeCartesianPath(pose_list_2, 0.05, 0.0, traj_msg, true);
			}
		}
		if (fraction < 0.97) {
			ROS_ERROR("PLANNING FAILURE!");
			double r,p,y;
			to_plan->trajectory_end.getBasis().getRPY(r,p,y);
			tf::Vector3 pos = to_plan->trajectory_end.getOrigin();
			ROS_ERROR("xyz: %f %f %f, rpy: %f %f %f", pos.x(),pos.y(),pos.z(),r,p,y);
			ROS_ERROR("Interested part: %s",ObjectTracker::get_interested_object().c_str());
			to_plan->planning_failure = true; //be generous?
			return;
		}

		if (to_plan->end_delay > 0) {
			ROS_INFO("Check after planning...");
			auto last_point = traj_msg.joint_trajectory.points.back();
			for (int i=0;i<last_point.velocities.size();++i) {
				ROS_INFO("Check 2 after planning...");
				last_point.velocities[i] = 0;
				last_point.accelerations[i] = 0;
			}
			ROS_INFO("Check after 3 planning...");
			traj_msg.joint_trajectory.points.push_back(last_point);
			traj_msg.joint_trajectory.points.back().time_from_start += ros::Duration(std::min(to_plan->end_delay/10.0,0.1));
			traj_msg.joint_trajectory.points.push_back(last_point);
			traj_msg.joint_trajectory.points.back().time_from_start += ros::Duration(to_plan->end_delay);
		}

		ROS_INFO("Check 4 after planning...");
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
				ROS_INFO_THROTTLE(1,"PLANNER -> Parallel control running");
				currently_planning = nullptr;
				boost::unique_lock<boost::mutex> current_plan_lock(current_plan_mutex);
				{ //mutex lock scope
					boost::unique_lock<boost::mutex> plan_lock(plan_mutex);
					while (plan_queue.size() == 0) {
						plan_condition.wait(plan_lock); //apparently this unlocks until awoken
					}
					currently_planning = plan_queue.front();
					plan_queue.pop_front();
					//don't notify here, only notify when adding
				} //unlock

				currently_planning->planning_status = PIPELINE_PROCESSING;
				if ((currently_planning->plan != nullptr) && (currently_planning->parent == nullptr)) { //don't do this if it has a parent?
					ROS_INFO("Plan already exists?");
					delete currently_planning->plan;
					currently_planning->plan = nullptr;
				}
				if (currently_planning->plan == nullptr) {
					if ((currently_planning->use_intermediate) || (currently_planning->use_trash) || (currently_planning->use_conveyor) || (currently_planning->scan)) {
						plan_joints(currently_planning);
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
	trajectory_msgs::JointTrajectoryPoint trashcan_points[2];
	trajectory_msgs::JointTrajectoryPoint conveyor_point;
	trajectory_msgs::JointTrajectoryPoint scan_point;
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

#endif
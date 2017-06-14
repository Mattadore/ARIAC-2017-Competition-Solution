#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <include/utility.h>
#include <include/planner.h>
#include <include/competition_interface.h>

//Controller class


class Controller {
public:
	Controller(Planner * planner_) : arm_control_group("manipulator"), control_thread(boost::bind(&Controller::parallel_control,this)),
	planner(planner_) {
		arm_control_group.setPoseReferenceFrame("/world");
		arm_control_group.setWorkspace(-20000,-20000,-20000,20000,20000,20000);
		std::vector<std_msgs::Float64> intermediate_configuration_vectors[2];
		force_stop = false;
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
		force_stop = true;
		arm_control_group.stop();
	}
protected:
	void grab_break() { //hardly thread-safe
		ROS_INFO("STARTING GRAB BREAK THREAD");
		arm_action::Ptr start_action = currently_executing;
		ros::Rate looprate(50);
		while (start_action == currently_executing) {
			if (CompetitionInterface::get_state(GRIPPER_ATTACHED) == BOOL_TRUE) {
				ROS_INFO_THROTTLE(1,"GRAB BREAK!");
				stop_controller();
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
		if (to_move->planning_failure) {
			return;
		}

		boost::thread * break_thread = nullptr;
		if (to_move->pick_part) {
			break_thread = new boost::thread(boost::bind(&Controller::grab_break,this));
		}
		CompetitionInterface::toggle_vacuum(to_move->vacuum_enabled);
		CompetitionInterface::set_arm_region(to_move->region);
		ros::Time start_time = ros::Time::now();
		arm_control_group.execute(*(to_move->plan));
		if (!force_stop) {
			ros::Duration elapsed = ros::Time::now() - start_time;
			ros::Duration intended = to_move->plan->trajectory_.joint_trajectory.points.back().time_from_start;
			if (elapsed < intended) {
				(intended-elapsed).sleep(); //ensure I take the correct amount of time if I undershoot
				//this is to keep prior planning consistent
			}
		}
		else {
			force_stop = false;
		}
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
				ROS_INFO_THROTTLE(1,"CONTROLLER -> Parallel control running");
				boost::unique_lock<boost::mutex> current_control_lock(current_control_mutex);
				currently_executing = nullptr;
				{ //mutex lock scope
					boost::unique_lock<boost::mutex> control_lock(control_mutex); //wait for lock on control queue
					while (control_queue.size() == 0) { 
						control_condition.wait(control_lock); //apparently this unlocks until awoken
					}
					currently_executing = control_queue.front();
					control_queue.pop_front();
					currently_executing->execution_status = PIPELINE_PROCESSING;
				} //unlock
				if (currently_executing->planning_status != PIPELINE_COMPLETE) { //ensure there is a plan to execute
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
	bool force_stop;
	std::list<arm_action::Ptr> control_queue; 
};

#endif
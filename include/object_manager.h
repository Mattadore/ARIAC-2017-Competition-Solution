#ifndef OBJECT_MANAGER_H_
#define OBJECT_MANAGER_H_

#include <algorithm>
#include <vector>
#include <utility>
#include <string>
#include <map>
#include <list>
#include <cstdio>
#include <cmath>

#include <ros/ros.h>

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
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer_server.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/regex.hpp> 
#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/conversions.h>

//tf_publisher_data data;
//bool gripper_attached = false;
//either filter or average part displacement to predicted
//treat OWN DATA as truth

//calculate the offset
//divide offset by min(max average num,parts added)?
//incorporate the offset into calculation
//transformation is calculated by:
//at a given time: frame + offset + special factors;
//change the 

//4951 -actual
//5492 -true
//7500 -size

//width 1: 
//11500
//12000

//width difference: ~ 1000

//on the ground: ~500

//upside down: 0.007516
//0.007000

//sinking, not wrong


namespace tracker {
	const unsigned char max_average_num = 45;
	const double noise_radius = 0.01; //idk something, set this later
	const double true_bin_height = 0.005000; //the true offset bin surface height (pain to calculate)
}
using namespace tracker;

enum part_location {
	LOCATION_NULL,
	LOCATION_ARM,
	LOCATION_BELT,
	LOCATION_AGV1,
	LOCATION_AGV2,
	LOCATION_GROUND,
	LOCATION_NUM
};


struct ObjectTypeData {
	std::string part_type_name;
	double size;
	double tf_base_offset;
	tf::Vector3 bin_start_point;
	tf::Vector3 bin_end_point;
	tf::Vector3 bin_start_rpy;
	tf::Vector3 conveyor_start_rpy;
	unsigned char bin_x_num;
	unsigned char bin_y_num;
	bool unique_orientation;
	double radius;
};

//TODO: find rough gripper size?
//TODO: find rough part radius?
struct grid_structure {
	double start_x;
	double start_y;
	char num_x;
	char num_y;
};

struct bin_data {
	std::string bin_name;
	ObjectTypeData * type_data;
	std::vector<grid_structure> structures_possible;
	grid_structure current_structure_belief;
	std::list<tf::Vector3> known_object_locations;
	tf::Pose object_rotation_transformation; //if below is not set, use bias in objecttypedata
	bool object_rotation_transformation_known; //lets code know if there is a known, measured rotation offset
												//todo: latch rotation offset to above value/
												//other values if close (depending on noise)

	//TODO: have a boolean or something if the above is valid. maybe it'll just get it from grid info, actually?
};



class ObjectTracker;

class ObjectData {
public:
	void reference_swap(std::string new_reference_name) {

	}
	void reset_filtering() {
		
	}
	std::string & get_name() {
		return part_name;
	}
	std::string & get_reference_frame() {
		return reference_frame;
	}
	tf::Vector3 get_velocity() {
		return relative_motion;
	}
	ObjectData(std::string name,std::string reference,ObjectTypeData * typedata,bool on_conveyor = false) {
		reference_frame = reference;
		part_name = name;
		type_information = typedata;
		average_list_index = 0;
		average_list_size = max_average_num;
		average_list_position_size = max_average_num;
		average_list_valid = 0;
		moving = on_conveyor;
		current_location = tf::StampedTransform(tf::Pose(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)),ros::Time::now(),reference,name);
		relative_motion = tf::Vector3(0,0,0);
		if (on_conveyor) {
			relative_motion = tf::Vector3(0.2,0,0);
		}
		else {
			relative_motion = tf::Vector3(0,0,0);
		}
	}
	ObjectTypeData * get_type_info() {
		return type_information;
	}
	//this code below is not user friendly
	void add_new_pose(tf::StampedTransform pose_in) {
		average_list[average_list_index] = pose_in;
		average_list_valid += (average_list_valid < average_list_size); //increment iff smaller than max
		tf::Quaternion result_q = average_list[average_list_index].getRotation();
		tf::Vector3 result_p = average_list[average_list_index].getOrigin();
		tf::Vector3 result_v = relative_motion;
		char total = 0;
		char i_last = average_list_index;
		// for (char i = (average_list_index-1+average_list_size)%average_list_size; i!=(average_list_index-average_list_valid+max_average_num)%average_list_size;i=(i-1+average_list_size)%average_list_size) {
		// 	++total;
		// 	ros::Duration inter_frame_time = average_list[i_last].stamp_ - average_list[i].stamp_;
		// 	if (inter_frame_time < ros::Duration(0.5)) { //just in case
		// 		tf::Vector3 next_v = tf::Vector3(0,(average_list[i_last].getOrigin().getY() - average_list[i].getOrigin().getY())/inter_frame_time.toSec(),0);
		// 		if (total == 1) {
		// 			result_v = next_v;
		// 		}
		// 		else {
		// 			//TODO: i feel like this would cause rounding error
		// 			//double weight = 1.0/(double)total;
		// 			//result_v = result_v.lerp(next_v,weight);
		// 			result_v += next_v;
		// 		}
		// 	}
		// 	i_last = i;
		// }
		// result_v = result_v / ((double)(average_list_valid-1));
		//result_v = tf::Vector3(0,0,0);
		total = 1;
		i_last = average_list_index;
		char max_iter = std::min(average_list_valid,average_list_position_size);
		for (char i = (average_list_index-1+average_list_size)%average_list_size; i!=(average_list_index-max_iter+average_list_size)%average_list_size;i=(i-1+average_list_size)%average_list_size) {
			++total;
			double inter_frame_time = ros::Duration(pose_in.stamp_ - average_list[i].stamp_).toSec();
			tf::Quaternion next_q = average_list[i].getRotation();
			tf::Vector3 next_p = average_list[i].getOrigin() + result_v*inter_frame_time;
			double weight = 1.0/(double)total;
			if (next_q.dot(result_q) < 0) {
				next_q = next_q.inverse();
			}
			result_q = result_q.slerp(next_q,weight);
			result_p = result_p.lerp(next_p,weight);
			i_last = i;
		}
		tf::Pose temp_location(result_q,result_p);
		tf::StampedTransform pose_out(pose_in);
		pose_out.setData(temp_location);

		current_location = pose_out;
		relative_motion = result_v;
		//relative_motion.setX(0);
		//relative_motion.setZ(0);

		average_list_index = (average_list_index+1) % average_list_size; //increment with possible loop back to start
	}

	// void add_new_pose(tf::StampedTransform pose_in) {
	// 	bool overwriting = (average_list_valid == average_list_size);
	// 	average_list_valid += (average_list_valid < average_list_size); //increment iff smaller than max
	// 	tf::StampedTransform old_pose = average_list[average_list_index];
	// 	average_list[average_list_index] = pose_in;
	// 	if (average_list_valid > 1) {
	// 		tf::Vector3 result_p = current_location.getOrigin();
	// 		tf::Quaternion result_q = pose_in.getRotation();
	// 		double weight = 1.0/((double)(average_list_valid));
	// 		if (overwriting) {
	// 			tf::Vector3 old_vector = old_pose.getOrigin();
	// 			result_p = result_p-weight*old_vector+weight*pose_in.getOrigin();
	// 		}
	// 		else {
	// 			result_p = result_p.lerp(pose_in.getOrigin(),weight);
	// 		}
	// 		tf::Pose temp_location(result_q,result_p);
	// 		pose_in.setData(temp_location);
	// 	}
	// 	current_location = pose_in;
	// 	average_list_index = (average_list_index+1) % max_average_num; //increment with possible loop back to start
	// }
	tf::StampedTransform get_current_location() {
		tf::StampedTransform out = current_location;
		out.stamp_ = ros::Time::now();
		double duration = ros::Duration(ros::Time::now() - current_location.stamp_).toSec();
		//because we're in relative frame, using velocity in relative frame, we transform before we combine part space
		out.setData(tf::Pose(tf::Quaternion(0,0,0,1),relative_motion*duration)*current_location);
		//ROS_INFO("%s: %f %f %f",part_name.c_str(),relative_motion.getX(),relative_motion.getY(),relative_motion.getZ());
		return out;
	}
	tf::StampedTransform get_location_at_time(ros::Time at_time) {
		tf::StampedTransform out = current_location;
		out.stamp_ = at_time;
		double duration = ros::Duration(at_time - current_location.stamp_).toSec();
		//because we're in relative frame, using velocity in relative frame, we transform before we combine part space
		out.setData(tf::Pose(tf::Quaternion(0,0,0,1),relative_motion*duration)*current_location);
		//ROS_INFO("%s: %f %f %f",part_name.c_str(),relative_motion.getX(),relative_motion.getY(),relative_motion.getZ());
		return out;
	}
	/*tf::StampedTransform get_global_location() {
		tf::StampedTransform to_reference = ObjectTracker::get_recent_transform("world",reference_frame);
		to_reference.setData(to_reference * get_current_location());
		return to_reference; 
	}*/
protected:
	//TODO: optimize this if it takes too long by just de-interpolating the last value from last calculation??

	std::string reference_frame;
	ObjectTypeData * type_information;
	tf::StampedTransform current_location;
	std::string part_name;
	tf::Vector3 relative_motion;
	bool moving;
	//handles dynamic moving average
	tf::StampedTransform average_list[max_average_num];
	unsigned char average_list_index;
	unsigned char average_list_size;
	unsigned char average_list_position_size;
	unsigned char average_list_valid;
};

class ObjectTracker {
public:

	//todo account for gripper size? ???

	static void initialize_tracker(ros::NodeHandle * nodeptr_,tf::TransformListener * listener_) {
		nodeptr = nodeptr_; 
		listener = listener_;
		initialize_object_types();
		subscribe("/tf", 100, &ObjectTracker::tf_callback);
		tf_publisher = nodeptr->advertise<tf2_msgs::TFMessage>("tf", 100);

	}
	static tf::StampedTransform get_recent_transform(std::string start, std::string end) {
		tf::StampedTransform transform_out;
		listener->lookupTransform(start,end,ros::Time(0),transform_out);
		return transform_out;
	}
	/*static tf::Pose get_current_end_location() {
		listener->lookupTransform("world","vacuum_gr",ros::Time(0),reference_transformation);
	}*/

	static void publish_tfs() {
		for (std::map<std::string,ObjectData *>::iterator it = lookup_map.begin(); it != lookup_map.end(); ++it) {
			tf::StampedTransform current_transform = it->second->get_current_location();
			tf2_msgs::TFMessage message_out;
			geometry_msgs::TransformStamped transformation_out;
			tf::transformStampedTFToMsg(current_transform,transformation_out);
			message_out.transforms.push_back(transformation_out);
			tf_publisher.publish(message_out);
		}
	}

	static void part_pick_occurred() {
		//do something when part pick occurs
	}

	static tf::StampedTransform get_gripper_pose(ros::Time time_in = ros::Time::now()) {
		tf::StampedTransform transform_out;
		listener->lookupTransform("world","vacuum_gripper_link",ros::Time(0),transform_out);
		return transform_out;
	}

	//gets the location and orientation you should use to grab the part
	//automatically adjusts if the part seems to be upside down
	static tf::Pose get_grab_pose(std::string part_name,ros::Time at_time = ros::Time::now(),double grab_offset = -0.014) {
		const tf::Quaternion identity(0,0,0,1);
		ObjectData & part_to_grab = *(lookup_map[part_name]);
		ObjectTypeData * part_typedata = part_to_grab.get_type_info();
		tf::Pose out_pose, object_true_location = part_to_grab.get_location_at_time(at_time);
		bool is_inverted = is_upside_down(object_true_location);
		double true_face_from_tf = true_bin_height - part_typedata->tf_base_offset;
		if (is_inverted) {
			tf::Pose flip_transform(tf::Quaternion(tf::Vector3(1,0,0),M_PI),tf::Vector3(0,0,0)); //flips the object pose
			tf::Pose handle_transform(identity,tf::Vector3(0,0,true_face_from_tf+grab_offset)); //specifies where the handle is
			out_pose = object_true_location * handle_transform * flip_transform;
		}
		else {
			true_face_from_tf += part_typedata->size; //grab from top, which is "size" away from the tf
			tf::Pose handle_transform(identity,tf::Vector3(0,0,true_face_from_tf-grab_offset)); //specifies where the handle is
			out_pose = object_true_location * handle_transform;
		}
		return out_pose;
	}
	static tf::Pose get_tray_pose(char agv_number) {
		if (agv_number == 1) {
			return tf::Pose(tf::Quaternion(0,0,1,0),tf::Vector3(0.3,3.15,0.75));
		}
		else if (agv_number == 2) {
			return tf::Pose(tf::Quaternion(0,0,0,1),tf::Vector3(0.3,-3.15,0.75));
		}
		else {
			ROS_ERROR("NO SUCH AGV %d",(int)agv_number);
			return tf::Pose();
		}
	}

	static bool part_exists(std::string part_name) {
		return lookup_map.count(part_name);
	}

	
protected:
	template<class M>
	static void subscribe(std::string name,uint32_t size,void(*fp)(M)) {
		subscriptions[name] = nodeptr->subscribe(name,size,fp);
	}

	static bool is_upside_down(tf::Pose check) {
		const tf::Vector3 z_vec(0,0,1);
		const tf::Quaternion identity(0,0,0,1);
		//gets what the "up" vector is for the pose
		tf::Vector3 z_local = (check * tf::Pose(identity,z_vec)).getOrigin()-check.getOrigin();
		double correlation = z_local.dot(z_vec); //compares with real "up"
		ROS_INFO("correlation: %f",correlation);
		return (correlation < -0.2); //arbitrary down correlation value
	}
	static void initialize_object_types() {
		type_data["disk_part"] = {"disk_part",0.021835,0.004951,tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,M_PI/4.0),tf::Vector3(0,0,0),2,2,true}; //correct
		type_data["gasket_part"] = {"gasket_part",0.020020,0.004951,tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,M_PI/4.0),tf::Vector3(0,0,0),2,2,true}; //correct
		type_data["gear_part"] = {"gear_part",0.008717,0.004951,tf::Vector3(0.1,0.1,0),
		tf::Vector3(0.5,0.5,0),tf::Vector3(0,0,0),tf::Vector3(0,0,0),4,4,true}; //correct
		type_data["piston_rod_part"] = {"piston_rod_part",0.007024,0.004951,tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,M_PI/4.0),tf::Vector3(0,0,0),2,2,true}; //correct
		type_data["pulley_part"] = {"pulley_part",0.072900,0.005500,tf::Vector3(0.15,0.15,0),
		tf::Vector3(0.45,0.45,0),tf::Vector3(0,0,M_PI/4.0),tf::Vector3(0,0,0),2,2,false};

		for (std::map<std::string,ObjectTypeData>::iterator i = type_data.begin();i!=type_data.end();++i) {
			object_data[i->first] = std::list<ObjectData>();
			ROS_INFO("%s",i->first.c_str());
		}
		//type_data["gasket_part"] = 
		//type_data["gasket_part"] = 
		//"gasket_part","gear_part","part1","part2","part3","part4","piston_rod_part","pulley_part"; 

	}

	static void tf_callback(const tf2_msgs::TFMessage & transform_list) {
		for (int tf_i = 0; tf_i < transform_list.transforms.size();++tf_i) {
			const geometry_msgs::TransformStamped & transformation = transform_list.transforms[tf_i];
			std::string parent = transformation.header.frame_id;
			std::string name = transformation.child_frame_id;
			//parses name
			boost::regex pattern("logical_camera_[0-9]+_((.*?part.*?)_(clone_)?[0-9]+)_frame", boost::regex::perl); 
			boost::smatch matches;
			std::string part_type, part_id, full_name;
			bool is_conveyor_part;
			if (boost::regex_match(name, matches, pattern)) {
				full_name = matches[1];
				part_type = matches[2];
				//part_id = matches[3];
				is_conveyor_part = (matches.size() == 4);
			}
			else {
				continue;
			}
			if (!lookup_map.count(full_name)) {
				ObjectData new_object(full_name,"world",&(type_data[part_type]),is_conveyor_part);
				object_data[part_type].push_back(new_object);
				lookup_map[full_name] = &(object_data[part_type].back());
			}
			tf::StampedTransform true_pose,reference_transformation;
			tf::transformStampedMsgToTF(transformation,true_pose);
			listener->lookupTransform(lookup_map[full_name]->get_reference_frame(),parent,ros::Time(0),reference_transformation);
			true_pose.setData(reference_transformation*true_pose);
			//ROS_INFO("%s",lookup_map[full_name]->get_reference_frame().c_str());
			true_pose.frame_id_ = lookup_map[full_name]->get_reference_frame();
			true_pose.child_frame_id_ = full_name;
			lookup_map[full_name]->add_new_pose(true_pose);
		}
	}
	static std::map<std::string,ObjectData *> lookup_map;
	static std::map<std::string,std::list<ObjectData> > object_data;
	static std::map<std::string,ObjectTypeData> type_data;
	static std::map<std::string,ros::Subscriber> subscriptions;
	static std::map<std::string,bin_data *> bin_data_lookup;
	static tf::TransformListener * listener;
	static ros::NodeHandle * nodeptr;
	static ros::Publisher tf_publisher;
};

std::map<std::string,ObjectData *> ObjectTracker::lookup_map;
std::map<std::string,std::list<ObjectData> > ObjectTracker::object_data;
std::map<std::string,ObjectTypeData> ObjectTracker::type_data;
std::map<std::string,ros::Subscriber> ObjectTracker::subscriptions;
tf::TransformListener * ObjectTracker::listener;
ros::NodeHandle * ObjectTracker::nodeptr;
ros::Publisher ObjectTracker::tf_publisher;

/*
int main(int argc, char ** argv) {
	ros::init(argc, argv, "advanced_tf_publisher");


	ros::rate(50) spin_rate;
	while (ros::ok()) {
		spin_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
*/

//Grid structure:
//list of possible grid alignments
//generate new grid alignments:
	//observation based
		//keep list of observations
			//positive pose observations + grab poses
			//negative pose observations + grab poses
	//have certain observations weighted more heavily
	//when we get back the location of a part, compare with expected
	
	//need method that, given a specific number of x and y parts as well as offsets,
	//returns a list of part locations, corresponding to what we'd expect
	//needs to work when there are multiple of a kind of part bin

	//needs a bin tracker with bin properties
	//bin structs maintain grids?
	//should we instead expand the object type struct?
	//would make the flow clearer

	//need to make object tracker track picked up parts
	//need to automatically link the likeliest picked up part when a part is picked up

	//give tenative generated possibilities lower weights
	
	//have both a bin tracker and a part type tracker, but use part type tracker primairly
	
//properties:
	//should be able to be fed in observations
	//should be able to return the "best" pickup pose, covers most bases
	//when determining which part to grab, possibly grab from lower bins if it seems that we're out of parts

	//whenever a part is picked up, update each possible grid's likelihood once the transform is known
	//try to find grid that best explains data
	//maybe have grid offsets, random permutations?
	//!!TREAT X AND Y ERROR SEPARATELY
	//Be mindful of minimum possible spacing:

	//give each alignment number a set of possible spacings
	//possible algorithm:
		//always pick a point that grabs from most likely grid
		//never pick a center part

		//pick second part from other side
		//try to pick most "informative" part

	//be aware of mirroring
	//only break mirroring if irrefutable evidence to the contrary

	//create comparator for different configurations

	//what do we know once we pick a part?
	
	//update weights based off of how similar to an observation, as compared to other parts
	//what is "acceptable" distance?

#endif
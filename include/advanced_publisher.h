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
#include <moveit/move_group_interface/move_group.h>
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

namespace tracker {
	const unsigned char max_average_num = 40;
	const double noise_radius = 0.01; //idk something, set this later
	const tf::Vector3 true_bin_height_offset = tf::Vector3(0,0,0.005490); //the true offset bin surface height (pain to calculate)
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

/*
const string pattern = "(abc)(def)";  
const string target = "abcdef"; 

boost::regex regexPattern(pattern, boost::regex::extended); 
boost::smatch what; 

bool isMatchFound = boost::regex_match(target, what, regexPattern); 
if (isMatchFound) 
{ 
    for (unsigned int i=0; i < what.size(); i++) 
    { 
        cout << "WHAT " << i << " " << what[i] << endl; 
    } 
} 
*/
/*
struct ObjectTrackStruct {
	std::string object_input_tf;
	std::string object_output_tf;
	std::string reference;
	//tf::StampedPose last_observation;
	tf::StampedPose anchor_pose; //pose for offset calculations
	tf::StampedPose recent_pose; //most recent published pose
	object_tracker_struct(unsigned char initial_location_type);
	unsigned char num_averaged;
	//make a method that can determine where everything is
	//have some thing to determine where everything starts?
	ObjectTrackStruct() { //pass in pose, where it is, what type of thing it's on, all that stuff
		num_averaged = 3;
	}
	void attach_to_arm() {
		reference = "/vacuum_gripper_link";
		num_averaged = 5; //weight of known value
	}
	void drop_on_agv1() {
		//reference = "/vacuum_gripper_link";
		//num_averaged = 5; //weight of known value
	}
	void drop_on_agv2() {

	}
}
*/




struct ObjectTypeData {
	std::string part_type_name;
	tf::Vector3 size;
	tf::Vector3 tf_base_offset;
	tf::Vector3 bin_start_point;
	tf::Vector3 bin_end_point;
	tf::Vector3 bin_start_rpy;
	unsigned char bin_x_num;
	unsigned char bin_y_num;
};

class ObjectData {
public:
	void reference_swap(std::string new_reference_name) {

	}
	std::string & get_name() {
		return part_name;
	}
	std::string & get_reference_frame() {
		return reference_frame;
	}
	ObjectData() {
		average_list_index = 0;
		average_list_size = max_average_num;
		average_list_valid = 0;
	}
	ObjectData(std::string name,std::string reference,ObjectTypeData * typedata) {
		reference_frame = reference;
		part_name = name;
		type_information = typedata;
		average_list_index = 0;
		average_list_size = max_average_num;
		average_list_valid = 0;
	}
	void add_new_pose(tf::StampedTransform pose_in) {
		average_list[average_list_index] = pose_in;
		average_list_valid += (average_list_valid < average_list_size); //increment iff smaller than max
		tf::Quaternion result_q = average_list[average_list_index].getRotation();
		tf::Vector3 result_v = average_list[average_list_index].getOrigin();
		char total = 1;
		for (char i = (average_list_index-1+average_list_size)%average_list_size; i!=(average_list_index-average_list_valid+max_average_num)%average_list_size;i=(i-1+average_list_size)%average_list_size) {
			//ROS_INFO("AVERAGING");
			++total;
			tf::Quaternion next_q = average_list[i].getRotation();
			tf::Vector3 next_v = average_list[i].getOrigin();
			double weight = 1.0/(double)total;
			if (next_q.dot(result_q) < 0) {
				next_q = next_q.inverse();
			}
			result_q = result_q.slerp(next_q,weight);
			result_v = result_v.lerp(next_v,weight);
		}
		tf::Pose temp_location(result_q,result_v);
		tf::StampedTransform pose_out(pose_in);
		pose_out.setData(temp_location);
		current_location = pose_out;

		average_list_index = (average_list_index+1) % average_list_size; //increment with possible loop back to start
	}

	// void add_new_pose(tf::StampedTransform pose_in) {
	// 	bool overwriting = (average_list_valid == average_list_size);
	// 	average_list_valid += (average_list_valid < average_list_size); //increment iff smaller than max
	// 	tf::StampedTransform old_pose = average_list[average_list_index];
	// 	average_list[average_list_index] = pose_in;
	// 	if (average_list_valid > 1) {
	// 		tf::Vector3 result_v = current_location.getOrigin();
	// 		tf::Quaternion result_q = pose_in.getRotation();
	// 		double weight = 1.0/((double)(average_list_valid));
	// 		if (overwriting) {
	// 			tf::Vector3 old_vector = old_pose.getOrigin();
	// 			result_v = result_v-weight*old_vector+weight*pose_in.getOrigin();
	// 		}
	// 		else {
	// 			result_v = result_v.lerp(pose_in.getOrigin(),weight);
	// 		}
	// 		tf::Pose temp_location(result_q,result_v);
	// 		pose_in.setData(temp_location);
	// 	}
	// 	current_location = pose_in;
	// 	average_list_index = (average_list_index+1) % max_average_num; //increment with possible loop back to start
	// }
	tf::StampedTransform get_current_location() {
		return current_location;
	}
protected:
	//TODO: optimize this if it takes too long by just de-interpolating the last value from last calculation??

	std::string reference_frame;
	ObjectTypeData * type_information;
	tf::StampedTransform current_location;
	std::string part_name;

	//handles dynamic moving average
	tf::StampedTransform average_list[max_average_num];
	unsigned char average_list_index;
	unsigned char average_list_size;
	unsigned char average_list_valid;
};

class ObjectTracker {
public:
	/*static void toggle_vacuum(bool _enabled) {
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
	}*/

	//todo account for gripper size?

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

protected:
	template<class M>
	static void subscribe(std::string name,uint32_t size,void(*fp)(M)) {
		subscriptions[name] = nodeptr->subscribe(name,size,fp);
	}
	static void initialize_object_types() {
		type_data["disk_part"] = {"disk_part",tf::Vector3(0,0,0.021835),tf::Vector3(0,0,0.004951),tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,M_PI/4.0),2,2}; //correct
		type_data["gasket_part"] = {"gasket_part",tf::Vector3(0,0,0.020020),tf::Vector3(0,0,0.004951),tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,M_PI/4.0),2,2}; //correct
		type_data["gear_part"] = {"gear_part",tf::Vector3(0,0,0.008717),tf::Vector3(0,0,0.004951),tf::Vector3(0.1,0.1,0),
		tf::Vector3(0.5,0.5,0),tf::Vector3(0,0,0),4,4}; //correct
		type_data["piston_rod_part"] = {"piston_rod_part",tf::Vector3(0,0,0.007024),tf::Vector3(0,0,0.004951),tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,M_PI/4.0),2,2}; //correct
		type_data["pulley_part"] = {"pulley_part",tf::Vector3(0.22,0.22,0.072907),tf::Vector3(0,0,0.005493),tf::Vector3(0.15,0.15,0),
		tf::Vector3(0.45,0.45,0),tf::Vector3(0,0,M_PI/4.0),2,2};
		//TODO: find values for these parts
		//type_data["part1"] = {"part1",tf::Vector3(0,0,0.009996),tf::Vector3(0,0,0.004997),tf::Vector3(0.1,0.1,0),
		//tf::Vector3(0.5,0.5,0),tf::Vector3(0,0,0),5,5};
		//type_data["part3"] = {"part3",tf::Vector3(0,0,0.009241),tf::Vector3(0,0,0.009615),tf::Vector3(0.2,0.2,0),
		//tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,0),5,5};
		type_data["part2"] = {"part2",tf::Vector3(0,0,0.009996),tf::Vector3(0,0,0.004997),tf::Vector3(0.1,0.1,0),
		tf::Vector3(0.5,0.5,0),tf::Vector3(0,0,0),5,5};
		type_data["part4"] = {"part4", tf::Vector3(0,0,0.009241),tf::Vector3(0,0,0.009615),tf::Vector3(0.2,0.2,0),
		tf::Vector3(0.4,0.4,0),tf::Vector3(0,0,0),5,5};

		for (std::map<std::string,ObjectTypeData>::iterator i = type_data.begin();i!=type_data.end();++i) {
			i->second.tf_base_offset -= true_bin_height_offset;
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
				//TODO: support conveyor belts
				ObjectData new_object(full_name,"world",&(type_data[part_type]));
				object_data[part_type].push_back(new_object);
				lookup_map[full_name] = &(object_data[part_type].back());
			}
			tf::StampedTransform true_pose,reference_transformation,current_transform;
			tf::transformStampedMsgToTF(transformation,true_pose);
			listener->lookupTransform(lookup_map[full_name]->get_reference_frame(),parent,ros::Time(0),reference_transformation);
			true_pose.setData(reference_transformation*true_pose);
			//ROS_INFO("%s",lookup_map[full_name]->get_reference_frame().c_str());
			true_pose.frame_id_ = lookup_map[full_name]->get_reference_frame();
			true_pose.child_frame_id_ = full_name;
			lookup_map[full_name]->add_new_pose(true_pose);
			current_transform = lookup_map[full_name]->get_current_location();
			tf2_msgs::TFMessage message_out;
			geometry_msgs::TransformStamped transformation_out;
			tf::transformStampedTFToMsg(current_transform,transformation_out);
			message_out.transforms.push_back(transformation_out);
			tf_publisher.publish(message_out);
		}
	}
	static std::map<std::string,ObjectData *> lookup_map;
	static std::map<std::string,std::list<ObjectData> > object_data;
	static std::map<std::string,ObjectTypeData> type_data;
	static std::map<std::string,ros::Subscriber> subscriptions;
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

// todo:
// subscribe to tf
// create map of stuff



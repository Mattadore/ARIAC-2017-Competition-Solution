#ifndef SEARCH_MANAGER_H_
#define SEARCH_MANAGER_H_


#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <include/competition_interface.h>
#include <include/object_manager.h>

struct part_bin_info {
	std::string part_name;
	unsigned char bin_num_min;
	unsigned char bin_num_max;
	bool unique_orientation;
	double radius;
	int index_counter;
};

struct bin_data {
public:
	bin_data(){}
	bin_data(std::string name,part_bin_info* data,tf::Vector3 location) {
		bin_name = name;
		bin_location = location;
		type_data = data;
		for (char x = data->bin_num_min;x<data->bin_num_max;++x) {
			for (char y = data->bin_num_min;y<data->bin_num_max;++y) {
				possible_grids.emplace_back(data->radius,x,y);
			}
		}
		//TODO: import grasp locations
	}
	// 	std::string bin_name;
	// part_bin_data * type_data;
	// std::list<grid_structure> possible_grids;
	// std::list<tf::Vector3> mirrored_object_locations;
	// std::list<tf::Vector3> grasp_locations;
	// tf::Vector3 bin_location; //TODO: maybe this should be a pose?
	// tf::Quaternion angle_pose_start;
	// double angle_offset_average = 0;
	// char observations = 0;
	bool grid_complete() {
		return (possible_grids.size() == 1);
	}
	int get_num_parts() {
		if (!grid_complete()) {
			ROS_ERROR("GRID NOT COMPLETE, PART NUM NOT KNOWN");
			return 0;
		}
		else {
			return possible_grids.front().num[0]*possible_grids.front().num[1];
		}
	}
	void populate_tracker() { //add part num to tracker
		//dostuff
		int size = get_num_parts();
		for (int ID = 1;ID <= size;++ID) {
			++(type_data->index_counter);
			tf::Vector3 offset = possible_grids.front().position_from_id(ID);
			tf::Vector3 true_position = offset+bin_location+tf::Vector3(0,0,ObjectTracker::part_type_generic_height(type_data->part_name));
			tf::StampedTransform new_object_pose;
			new_object_pose.setData(tf::Pose(get_rotation(),true_position)); //yess
			new_object_pose.stamp_ = ros::Time::now();

			ObjectTracker::add_bin_part(type_data->part_name, type_data->index_counter, new_object_pose);
		}
	}
	void add_observation(tf::Pose pose_in, char id_number = 0) {
		// if (grid_complete()) { //for safety tbh
		// 	return;
		// }
		//TODO: make this safer
		//do pose correction to point the same way
		tf::Pose pose_corrected = pose_in;
		tf::Vector3 z_axis(0,0,1);
		tf::Vector3 pose_in_z = (pose_in(z_axis)-pose_in.getOrigin()).normalize(); //probably don't need to normalize
		if (std::abs(z_axis.dot(pose_in_z)) < 1) {
			tf::Vector3 cross_axis = pose_in_z.cross(z_axis).normalize(); //hope I get this right
			pose_corrected.setRotation(pose_in.getRotation()*tf::Quaternion(cross_axis,z_axis.angle(pose_in_z)));
		}

		//now in local coords
		pose_corrected.setOrigin(pose_corrected.getOrigin()-bin_location);
		//TODO: more pose correction, if necessary

		if (observations == 0) {
			angle_pose_start = pose_corrected.getRotation();
		}
		else {
			double angle_offset = angle_pose_start.angleShortestPath(pose_corrected.getRotation());
			angle_offset_average *= ((double)observations)/((double)observations+1);
			angle_offset_average += angle_offset*(1.0/((double)observations+1));
		}
		//puts all mirrors as grasps
		tf::Vector3 object_location = pose_in.getOrigin();
		mirrored_object_locations.push_back((object_location-bin_location)*tf::Vector3(-1,1,1));
		mirrored_object_locations.push_back((object_location-bin_location)*tf::Vector3(1,-1,1));
		mirrored_object_locations.push_back((object_location-bin_location)*tf::Vector3(-1,-1,1));
		//filter the models
		for (std::list<grid_structure>::iterator iter = possible_grids.begin();iter!=possible_grids.end();) {
			if (!iter->incorporate(pose_corrected,id_number,(observations == 1))) {
				iter = possible_grids.erase(iter);
			}
			else {
				++iter;
			}
		}

		++observations;

		if (grid_complete()) {
			populate_tracker();
			//publish all as tfs, i suppose
		}
		else if (possible_grids.size() == 0) {
			ROS_ERROR("ALL MODELS FAILED ON BIN %s",bin_name.c_str());
		}
	}
	tf::Quaternion get_rotation() {
		if (observations == 0) {
			ROS_ERROR("No observations, cannot determine angle");
		}
		tf::Quaternion angle = angle_pose_start + tf::Quaternion(tf::Vector3(0,0,1),angle_offset_average);
		return angle;
	}
	void pop_search_location() {
		if (!mirrored_object_locations.empty()) {
			mirrored_object_locations.pop_front();
		}
		else if (!grasp_locations.empty()) {
			grasp_locations.pop_front();
		}
		else {
			ROS_ERROR("NO MORE GRASP LOCATIONS TO POP");
		}
	}
	tf::Vector3 get_search_location() { //if we recently found a part, return those. Otherwise, use generic ones.
		tf::Vector3 return_v;
		if (!mirrored_object_locations.empty()) {
			return_v = mirrored_object_locations.front();
		}
		else if (!grasp_locations.empty()) {
			return_v = grasp_locations.front();
		}
		else {
			ROS_ERROR("NO MORE GRASP LOCATIONS");
		}
		return_v += bin_location;
		return_v.setZ(ObjectTracker::part_type_generic_grab_height(type_data->part_name));
		return return_v;
	}
	bool has_search_location() { //if we recently found a part, return those. Otherwise, use generic ones.
		return ((mirrored_object_locations.empty())&&(grasp_locations.empty()));
	}
protected:
	//bin size is 0.6
	//only rescale axes when pertinent observations come in
	struct grid_structure {
		//0 is x 1 is y
		double scale[2] = {0,0};
		char num[2];
		double radius;
		char elements[2] = {0,0};
		grid_structure(double rad,char x, char y) {
			radius = rad;
			num[0] = x;
			num[1] = y;
		}
		//fills iterates x outer, y inside
		char axis_index_from_id(char axis,char number) {
			if (axis == 0) {
				return ((number-1)/num[0])+1;
			}
			else {
				return ((number-1)%num[1])+1;
			}
		}
		inline tf::Vector3 position_from_id(char number) {
			return position_from_index(axis_index_from_id(0,number),axis_index_from_id(1,number));
		}	
		inline bool is_central(char axis, char id) {
			return (((num[axis]%2)==1)&&(id==(num[axis]/2+1))); //if odd and in the center
		}
		inline tf::Vector3 position_from_index(char x,char y) {
			return tf::Vector3(scale[0]*get_index_scaling(0,x),scale[1]*get_index_scaling(1,y),0);
		}
		inline double get_index_scaling(char axis,char index) {
			return ((double)index)-((num[axis]+1)/2.0);
		}
		bool is_edge(char id) {
			for (char axis=0;axis<=1;++axis) {
				char index = axis_index_from_id(axis,id);
				if ((index == 1) || (index == num[axis])) {
					return true;
				}
			}
			return false;
		}
		bool incorporate(tf::Pose & location, char id_number = 0,bool edge = false) {
			if (id_number != 0) { //if we know the id

				//set unset values
				//compare true value to false value, check if outside a range of radius from where should be
				//scale always more than radius
				tf::Vector3 true_location = location.getOrigin();
				double temp_scale[2] = {0,0};
				if (edge && (!is_edge(id_number))) { //edge mismatch
					return false;
				}
				for (char axis=0;axis<=1;++axis) {
					if (is_central(axis,id_number)) { //probably more to put here, I imagine?
						if (std::abs(true_location.m_floats[axis]) > radius) { //outta bounds, bro
							return false;
						}
					}
					else {
						double scaling_factor = get_index_scaling(axis,axis_index_from_id(axis,id_number));
						//calculate the scaling for this input
						temp_scale[axis] = true_location.m_floats[axis]/scaling_factor;

						//do a bunch of checks
						if (temp_scale[axis] < 2*radius){ //checks both the quadrant and the minimum spacing
							return false;
						}
						if ((temp_scale[axis] * (num[axis]-1)) > (0.6-2*radius)) { //checks max spacing
							return false;
						}
						if (elements[axis] != 0) { //EXISTING GRID
							double expected_location = scaling_factor * scale[axis];
							if (std::abs(expected_location-true_location.m_floats[axis]) > radius) { //does not align to grid
								return false;
							}
							//update average
							scale[axis] *= ((double)elements[axis])/((double)elements[axis]+1.0);
							scale[axis] += (1.0/((double)elements[axis]+1)) * temp_scale[axis];
						}
						else {
							scale[axis] = temp_scale[axis];
						}
						++elements[axis];
					}
				}
			}
			return true;
		}
	};
	std::string bin_name;
	part_bin_info * type_data;
	tf::Vector3 bin_location; //TODO: maybe this should be a pose?
	std::list<grid_structure> possible_grids;
	std::list<tf::Vector3> mirrored_object_locations;
	std::list<tf::Vector3> grasp_locations;
	tf::Quaternion angle_pose_start;
	double angle_offset_average = 0;
	char observations = 0;
	//TODO: have a boolean or something if the above is valid. maybe it'll just get it from grid info, actually?
};

//assumes we always make grasp attempts at identity? orientation
class SearchManager {
public:
	SearchManager() {
		part_bin_data["disk_part"] = {"disk_part",1,3,true,0.06,0};
		part_bin_data["gasket_part"] = {"gasket_part",1,3,true,0.05,0};
		part_bin_data["gear_part"] = {"gear_part",1,5,true,0.04,0};
		part_bin_data["piston_part"] = {"piston_part",1,4,true,0.03,0};
		part_bin_data["pulley_part"] = {"pulley_part",1,2,false,0.1,0};

		bin_locations["bin1"] = tf::Vector3(-1.0,-1.33,0);
		bin_locations["bin2"] = tf::Vector3(-1.0,-0.535,0);
		bin_locations["bin3"] = tf::Vector3(-1.0,0.23,0);
		bin_locations["bin4"] = tf::Vector3(-1.0,0.995,0);
		bin_locations["bin5"] = tf::Vector3(-0.3,-1.33,0);
		bin_locations["bin6"] = tf::Vector3(-0.3,-0.535,0);
		bin_locations["bin7"] = tf::Vector3(-0.3,0.23,0);
		bin_locations["bin8"] = tf::Vector3(-0.3,0.995,0);

		std::vector<std::string> parts {"gear_part","piston_rod_part","pulley_part","disk_part","gasket_part"};
		for (std::string & part : parts) {
			std::vector<std::string> locations = CompetitionInterface::get_material_locations(part);
			part_locations[part] = locations;
			for (std::string & location : locations) {
				bin_part_types[location] = part;
			}
		}
		//create all relevant bins
		for (auto & index : bin_part_types) {
			bin_lookup.emplace(index.first,bin_data(index.first,&(part_bin_data[index.second]),bin_locations[index.first]));
		}
	}

	bool unfound_parts(std::string part_type) {
		std::vector<std::string> & bin_list = part_locations[part_type];
		for (std::string & bin_name : bin_list) {
			if (!bin_lookup[bin_name].grid_complete()) {
				return true;
			}
		}
		return false;
	}

	tf::Vector3 search(std::string part_type) {
		std::string bin_name = get_current_search_bin(part_type);
		return bin_lookup[bin_name].get_search_location();
	}

	void search_success(std::string part_type) { //invoke after finishing part scan
		std::string object_name = ObjectTracker::get_held_object();
		if (object_name == "") {
			ROS_ERROR("no held object");
		}
		std::string bin_name = get_current_search_bin(part_type);
		tf::Pose grasp_pose = tf::Pose(tf::Quaternion(0,0,0,1),bin_lookup[bin_name].get_search_location());
		tf::Pose offset = ObjectTracker::get_internal_transform(object_name);

	}

	std::string get_current_search_bin(std::string part_type) {
		std::vector<std::string> & bin_list = part_locations[part_type];
		for (std::string & bin_name : bin_list) {
			if (!bin_lookup[bin_name].grid_complete()) {
				return bin_name;
			}
		}
		ROS_ERROR("NO MORE SEARCH LOCATIONS FOR %s AT ALL :(",part_type.c_str());
		return "";
	}

	void search_fail(std::string part_type) { //pops the search spot location
		//TODO: negative confirmation
		std::string bin_name = get_current_search_bin(part_type);
		bin_lookup[bin_name].pop_search_location();
	}

protected:
	std::string get_bin_to_search(std::string part_type) {
		for (std::string & bin_name : (part_locations[part_type])) { //from first to last

		}
	}
	std::map<std::string,part_bin_info> part_bin_data; //indexed by part type
	std::map<std::string,bin_data> bin_lookup; //indexed by bin name

	std::map<std::string,std::vector<std::string> > part_locations; //ITERATE FROM FRONT
	std::map<std::string,std::string> bin_part_types; //indexed by bin name
	std::map<std::string,tf::Vector3> bin_locations; //indexed by bin name

};

#endif
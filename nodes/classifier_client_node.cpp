#include <ros/ros.h>
#include <iostream>
#include <avatar_locomanipulation/BinaryClassifierQuery.h>

#include <avatar_locomanipulation/models/robot_model.hpp>

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

#define CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE 0
#define CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE 1

#define CONTACT_TRANSITION_DATA_LEFT_HAND 0
#define CONTACT_TRANSITION_DATA_RIGHT_HAND 1
#define CONTACT_TRANSITION_DATA_BOTH_HANDS 2


void addToXVector(const Eigen::Vector3d & pos, const Eigen::Quaterniond & ori, std::vector<double> & x){
	Eigen::AngleAxisd tmp_ori(ori.normalized()); // gets the normalized version of ori and sets it to an angle axis representation
	Eigen::Vector3d tmp_ori_vec3 = tmp_ori.axis()*tmp_ori.angle();

	for(int i = 0; i < pos.size(); i++){
		x.push_back(pos[i]);
	}
	for(int i = 0; i < tmp_ori_vec3.size(); i++){
		x.push_back(tmp_ori_vec3[i]);
	}
}


void populateXVector(std::vector<double> & x, 
	const double & stance_origin, const double & manipulation_type,
	const Eigen::Vector3d & swing_foot_start_pos, const Eigen::Quaterniond & swing_foot_start_ori,  
	const Eigen::Vector3d & pelvis_pos, const Eigen::Quaterniond & pelvis_ori,  
	const Eigen::Vector3d & landing_foot_pos, const Eigen::Quaterniond & landing_foot_ori,  
	const Eigen::Vector3d & right_hand_start_pos, const Eigen::Quaterniond & right_hand_start_ori,  
	const Eigen::Vector3d & left_hand_start_pos, const Eigen::Quaterniond & left_hand_start_ori){

	x.push_back(stance_origin);
	x.push_back(manipulation_type);
	addToXVector(swing_foot_start_pos, swing_foot_start_ori, x);
	addToXVector(pelvis_pos, pelvis_ori, x);
	addToXVector(landing_foot_pos, landing_foot_ori, x);
	addToXVector(right_hand_start_pos, right_hand_start_ori, x);
	addToXVector(left_hand_start_pos, left_hand_start_ori, x);
}

void test(ros::ServiceClient & client){
	// ~/Data/param_set_1/right_hand/transitions_data_with_task_space_info/positive_examples/right_hand_left_foot_s200_1.yaml
	double stance_origin = CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE;
	double manipulation_type = CONTACT_TRANSITION_DATA_RIGHT_HAND;

	Eigen::Vector3d swing_foot_start_pos(0.15411,-0.2074,0.0);
	Eigen::Quaterniond swing_foot_start_ori(0.998433,0,0,-0.0559473);

	Eigen::Vector3d pelvis_pos(-0.04466, -0.0581, 0.9522);
	Eigen::Quaterniond pelvis_ori(0.999608,0,0,-0.0279);

	Eigen::Vector3d landing_foot_pos(0.08006, -0.3344725,0);
	Eigen::Quaterniond landing_foot_ori(0.9746387,0,0,-0.22378);

	Eigen::Vector3d right_hand_start_pos(0.37345209, -0.67836299, 1.0333);
	Eigen::Quaterniond right_hand_start_ori(0.778100, 0.22986, -0.234809 ,0.535339);

	Eigen::Vector3d left_hand_start_pos(0,0,0);
	Eigen::Quaterniond left_hand_start_ori(1,0,0,0);


	avatar_locomanipulation::BinaryClassifierQuery srv;

	// std::vector<double> x;
	int input_dim = 32;
	srv.request.x.clear();
	srv.request.x.reserve(input_dim);

	populateXVector(srv.request.x, stance_origin, manipulation_type,
					    		   swing_foot_start_pos, swing_foot_start_ori,
					   			   pelvis_pos, pelvis_ori,
								   landing_foot_pos,landing_foot_ori,
								   right_hand_start_pos, right_hand_start_ori,
								   left_hand_start_pos, left_hand_start_ori);


	// Result
	double prediction;

	// Make request at 10kHz
	ros::Rate r(10000); // 10,000 hz

    auto t1 = Clock::now();
    auto t2 = Clock::now();
	double time_span;

	while (ros::ok()){
		t1 = Clock::now();
		if (client.call(srv)){
			prediction = srv.response.y;
			// ROS_INFO("Prediction: %0.4f", srv.response.y);
		}else{
		    ROS_ERROR("Failed to call service locomanipulation_feasibility_classifier");
		}
		t2 = Clock::now();
		time_span = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();

		std::cout << "test Pred = " << prediction << std::endl;
		std::cout << " test inference round trip time = " << time_span << " seconds" << std::endl;
		std::cout << " test frequency = " << (1.0/time_span) << " Hz" << std::endl;
		r.sleep();
	}


}

int main(int argc, char ** argv){   
	ros::init(argc, argv, "classifier_client");
	std::string service_name = "locomanipulation_feasibility_classifier";

	std::cout << "Initializing client for " << service_name << std::endl;

	// Create ROS handle and client
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<avatar_locomanipulation::BinaryClassifierQuery>(service_name);

	// Wait for the service
	std::cout << "waiting for " << service_name << " service..." << std::endl;
	ros::service::waitForService(service_name);
	std::cout << "service available" << std::endl;

	// Prepare request
	avatar_locomanipulation::BinaryClassifierQuery srv;
	int dim = 32;
	srv.request.x.reserve(dim);
	for(int i = 0; i < dim; i++){
		srv.request.x.push_back(0);
	}
	double prediction;


	test(client);

	// Make request at 10kHz
	// ros::Rate r(10000); // 10,000 hz


 //    auto t1 = Clock::now();
 //    auto t2 = Clock::now();
	// double time_span;

	// while (ros::ok()){
	// 	t1 = Clock::now();
	// 	if (client.call(srv)){
	// 		prediction = srv.response.y;
	// 		// ROS_INFO("Prediction: %0.4f", srv.response.y);
	// 	}else{
	// 	    ROS_ERROR("Failed to call service locomanipulation_feasibility_classifier");
	// 	    return 1;		
	// 	}
	// 	t2 = Clock::now();
	// 	time_span = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();

	// 	std::cout << "Pred = " << prediction << std::endl;
	// 	std::cout << " inference round trip time = " << time_span << " seconds" << std::endl;
	// 	std::cout << " frequency = " << (1.0/time_span) << " Hz" << std::endl;
	// 	r.sleep();
	// }

	return 0;
}

/*

ideal interface:
	double get_prediction(current_node, neighbor_node)

	if hand is stationary  (1 call to the classifier)
		we just call the classifier

	x = number of bin discretizations for the hand trajectory

	if the hand is moving with stepping (x calls to the classifier)
		discretize the hand trajectory into x bins
		for each bin, call the classifier with the stepping motion

	if the hand is stationary (2x calls to classifier)
		discretize the hand trajectory into x bins
		for each bin, call the classifier with each two steps


	current_node 
		3 pelvis_ori = average of midfeet positions
		3 pelvis_position = xy = average of midfeet positions.
		  pelvis_position.z = constant height

		3 swing foot position
		3 swing foot orientation

		s_variable gives:
		1 manipulation_type

		3 hand_position
		3 hand_orientation

		other hand
		3 hand_position.setZero()
		3 hand_ori.setIdentity()

	neighbor_node
		1 stance_type	
		3 landing_foot_location
		3 landing foot orientation


request must be formatted to:

	self.stance_origin_to_num = { "left_foot": 0, "right_foot":1 }
	self.manipulation_type_to_num = { "left_hand": 0, "right_hand":1, "both_hands":2 }

	self.x = np.concatenate( (stance_origin_type,manipulation_type,
							  self.swing_foot_starting_position, self.swing_foot_starting_orientation_vec,
							  self.pelvis_starting_position, self.pelvis_starting_orientation_vec,
							  self.landing_foot_position, self.landing_foot_orientation_vec,
							  self.right_hand_starting_position,self.right_hand_starting_orientation_vec,
							  self.left_hand_starting_position,self.left_hand_starting_orientation_vec) )


*/
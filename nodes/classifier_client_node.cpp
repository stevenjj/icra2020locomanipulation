#include <ros/ros.h>
#include <iostream>
#include <avatar_locomanipulation/BinaryClassifierQuery.h>


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

	// Make request
	if (client.call(srv)){
		ROS_INFO("Prediction: %0.4f", srv.response.y);
	}else{
	    ROS_ERROR("Failed to call service locomanipulation_feasibility_classifier");
	    return 1;		
	}

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
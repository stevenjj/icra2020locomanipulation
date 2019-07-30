#include <avatar_locomanipulation/cubic_interpolation_module/six_dim.hpp>


SixDim::SixDim(){
	output.clear();
	for (int i = 0; i < 6; i++){
		output.push_back(0.0);
	}
}


SixDim::SixDim(const int & first_waypoint, const std::string & filename_input){
	// Initializa Param Handler
	ParamHandler param_handler;
	// Load the yaml file
	param_handler.load_yaml_file(filename_input);
	// Holds the waypoint_#
	char point[12];
	// Holds the waypoints for each dimension
	std::vector<double> xs, ys, zs, rxs, rys, rzs;
	// Temporarily holds waypoints from getNestedValue
	double x, y, z, rx, ry, rz, rw;

	// Grab the waypoint and pose data from the yaml file
	for(int i=first_waypoint; i<(first_waypoint+4); ++i){
		snprintf(point, sizeof(char)*32, "waypoint_%d", i);
		param_handler.getNestedValue({point, "x"}, x);
		param_handler.getNestedValue({point, "y"}, y);
		param_handler.getNestedValue({point, "z"}, z);

		xs.push_back(x);
		ys.push_back(y);
		zs.push_back(z);

		param_handler.getNestedValue({point, "rx"}, rx);
		param_handler.getNestedValue({point, "ry"}, ry);
		param_handler.getNestedValue({point, "rz"}, rz);
		param_handler.getNestedValue({point, "rw"}, rw);

		Eigen::Quaterniond quat;
		Eigen::Vector3d axisangle;

		// Build quaternion from the waypoints
		quat.x() = rx;
		quat.y() = ry;
		quat.z() = rz;
		quat.w() = rw;

		// convert this to axisangle Vector3d
		math_utils::convert(quat, axisangle);

		rxs.push_back(axisangle[0]);
		rys.push_back(axisangle[1]);
		rzs.push_back(axisangle[2]);
		
	}

	// Create our 6 OneDim Interpolations
	fx = std::shared_ptr<OneDim>(new OneDim(xs) );
	fy = std::shared_ptr<OneDim>(new OneDim(ys) );
	fz = std::shared_ptr<OneDim>(new OneDim(zs) );
	frx = std::shared_ptr<OneDim>(new OneDim(rxs) );
	fry = std::shared_ptr<OneDim>(new OneDim(rys) );
	frz = std::shared_ptr<OneDim>(new OneDim(rzs) );

	// Initialize output vector of doubles
	output.clear();
	for (int i = 0; i < 6; i++){
		output.push_back(0.0);
	}

	// std::cout << "[SixDim] Created" << std::endl;
}


SixDim::~SixDim(){

}


void SixDim::evaluate(const double & s_local){
 	double s_in; 
 	s_in = s_local;

 	// Update output vector
	output[0] = fx->evaluate(s_in);
	output[1] = fy->evaluate(s_in);
	output[2] = fz->evaluate(s_in);
	output[3] = frx->evaluate(s_in);
	output[4] = fry->evaluate(s_in);
	output[5] = frz->evaluate(s_in);

}
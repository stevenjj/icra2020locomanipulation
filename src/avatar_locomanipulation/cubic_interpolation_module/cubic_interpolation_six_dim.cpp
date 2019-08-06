#include <avatar_locomanipulation/cubic_interpolation_module/cubic_interpolation_six_dim.hpp>


CubicInterpolationSixDim::CubicInterpolationSixDim(){
	output.clear();
	output.reserve(6);
	for (int i = 0; i < 6; i++){
		output[i] = 0.0;
	}
}


CubicInterpolationSixDim::CubicInterpolationSixDim(const int & first_waypoint, const std::string & filename_input){
	// Initializa Param Handler
	ParamHandler param_handler;
	// Load the yaml file
	param_handler.load_yaml_file(filename_input);
	// String to hold the waypoint_#
	std::string waypoint_string("waypoint_");
	// Holds the waypoints for each dimension
	std::vector<double> xs, ys, zs, rxs, rys, rzs;
	xs.reserve(4); ys.reserve(4); zs.reserve(4); 
	rxs.reserve(4); rys.reserve(4); rzs.reserve(4); 
	// Temporarily holds waypoints from getNestedValue
	double x, y, z, rx, ry, rz, rw;
	// Allows us to index into xs, ys, etc
	int j=0;


	// Grab the waypoint and the following 3 other waypoints' pose data from the yaml file
	for(int i=first_waypoint; i<(first_waypoint+4); ++i){
		waypoint_string = "waypoint_" + std::to_string(i);
		param_handler.getNestedValue({waypoint_string, "x"}, x);
		param_handler.getNestedValue({waypoint_string, "y"}, y);
		param_handler.getNestedValue({waypoint_string, "z"}, z);

		xs[j] = x;
		ys[j] = y;
		zs[j] = z;

		param_handler.getNestedValue({waypoint_string, "rx"}, rx);
		param_handler.getNestedValue({waypoint_string, "ry"}, ry);
		param_handler.getNestedValue({waypoint_string, "rz"}, rz);
		param_handler.getNestedValue({waypoint_string, "rw"}, rw);

		Eigen::Quaterniond quat;
		Eigen::Vector3d axisangle;

		// Build quaternion from the waypoints
		quat.x() = rx;
		quat.y() = ry;
		quat.z() = rz;
		quat.w() = rw;

		// convert this to axisangle Vector3d
		math_utils::convert(quat, axisangle);

		rxs[j] = axisangle[0];
		rys[j] = axisangle[1];
		rzs[j] = axisangle[2];
		
		++j;
	}

	// Create our 6 CubicInterpolationOneDim Interpolations
	fx = std::shared_ptr<CubicInterpolationOneDim>(new CubicInterpolationOneDim(xs) );
	fy = std::shared_ptr<CubicInterpolationOneDim>(new CubicInterpolationOneDim(ys) );
	fz = std::shared_ptr<CubicInterpolationOneDim>(new CubicInterpolationOneDim(zs) );
	frx = std::shared_ptr<CubicInterpolationOneDim>(new CubicInterpolationOneDim(rxs) );
	fry = std::shared_ptr<CubicInterpolationOneDim>(new CubicInterpolationOneDim(rys) );
	frz = std::shared_ptr<CubicInterpolationOneDim>(new CubicInterpolationOneDim(rzs) );

	// Initialize output vector of doubles
	output.clear();
	output.reserve(6);
	for (int i = 0; i < 6; i++){
		output[i] = 0.0;
	}

	// std::cout << "[CubicInterpolationSixDim] Created" << std::endl;
}


CubicInterpolationSixDim::~CubicInterpolationSixDim(){

}


void CubicInterpolationSixDim::evaluate(const double & s_local){
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
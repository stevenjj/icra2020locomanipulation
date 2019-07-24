#include <avatar_locomanipulation/cubic_interpolation_module/seven_dim.hpp>


SevenDim::SevenDim(){

}


SevenDim::SevenDim(const int & first_waypoint, const std::string & yaml_name){

	// Initializa Param Handler
	ParamHandler param_handler;
	std::string filename = THIS_PACKAGE_PATH"hand_trajectory/" + yaml_name;
	// Load the yaml file
	param_handler.load_yaml_file(filename);
	// Holds the waypoint_#
	char point[12];
	// Holds the waypoints for each dimension
	std::vector<double> xs, ys, zs, rxs, rys, rzs, rws;
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

		rxs.push_back(rx);
		rys.push_back(ry);
		rzs.push_back(rz);
		rws.push_back(rw);
		
	}

	// Create our 7 OneDim Interpolations
	fx = std::shared_ptr<OneDim>(new OneDim(xs) );
	fy = std::shared_ptr<OneDim>(new OneDim(ys) );
	fz = std::shared_ptr<OneDim>(new OneDim(zs) );
	frx = std::shared_ptr<OneDim>(new OneDim(rxs) );
	fry = std::shared_ptr<OneDim>(new OneDim(rys) );
	frz = std::shared_ptr<OneDim>(new OneDim(rzs) );
	frw = std::shared_ptr<OneDim>(new OneDim(rws) );

	std::cout << "[SevenDim] Created" << std::endl;
}


SevenDim::~SevenDim(){

}


void SevenDim::evaluate(const double & s_local){
 	double s_in = s_local;

 	std::cout << "\nx_pos: " << std::endl;
	fx->evaluate(s_in);
	std::cout << "\ny_pos: " << std::endl;
	fy->evaluate(s_in);
	std::cout << "\nz_pos: " << std::endl;
	fz->evaluate(s_in);
	std::cout << "\nx_ori: " << std::endl;
	frx->evaluate(s_in);
	std::cout << "\ny_ori: " << std::endl;
	fry->evaluate(s_in);
	std::cout << "\nz_ori: " << std::endl;
	frz->evaluate(s_in);
	std::cout << "\nw_ori: " << std::endl;
	frw->evaluate(s_in);
}
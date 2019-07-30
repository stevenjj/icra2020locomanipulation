#include <avatar_locomanipulation/cubic_interpolation_module/six_dim_vec.hpp>


SixDimVec::SixDimVec(){

}


SixDimVec::SixDimVec(const std::string & filename_input){
	filename = filename_input;

	// Initializa Param Handler
	ParamHandler param_handler;
	// Load the yaml file
	param_handler.load_yaml_file(filename);
	// Get number of waypoitns
	param_handler.getInteger("num_waypoints", N);

	for(int i=1; i<(N-2); ++i){
		temp = std::shared_ptr<SixDim>(new SixDim(i, filename) );
		six_dim_vec.push_back(temp);
	}

	// std::cout << "[SixDimVec] Created" << std::endl;
	// std::cout << "six_dim_vec.size() " << six_dim_vec.size() << std::endl;
}


SixDimVec::~SixDimVec(){

}


void SixDimVec::evaluate(const double & s_global){
	// Clamp 0.0 <= s <= 1.0
	double s_ = clamp(s_global);

	for(int i=0; i<(N-2); ++i){
		// Ensure we use global s to select the proper SixDim to use
		if( ( i / ((double) (N)) ) <= s_ && s_ < ( ((double) (i+3))/((double) (N-1)) ) ){

			// Use the global s to obtain the local s as used by each OneDim
			// s_local = (1/(smax - smin))*(sg - smin)
			double s_local = ( 1 / ( ( ((double) (i+3))/((double) (N-1)) ) - ( i / ((double) (N)) ) )) * ( s_ - ( i / ((double) (N)) ) );
			
			// Feed evaluate the local s value
			six_dim_vec[i]->evaluate(s_local);
			temp = six_dim_vec[i];
			break;
		}
	} 
	if (s_ >= 1.0){
		six_dim_vec[N-3]->evaluate(s_);	
		temp = six_dim_vec[N-3];	
	}

	convertToQuat();

	// std::cout << "\nx_out:\n" << pos_out[0];
	// std::cout << "\ny_out:\n" << pos_out[1];
	// std::cout << "\nz_out:\n" << pos_out[2];
	// std::cout << "\n rx_out, ry_out, rz_out, rw_out\n(";
	// math_utils::printQuat(quat_out); 

}

void SixDimVec::getPose(const double & s_global, Eigen::Vector3d & position_out, Eigen::Quaterniond & orientation_out){
	evaluate(s_global);
	position_out = pos_out;
	orientation_out = quat_out;
}


double SixDimVec::clamp(const double & s_in){
    if (s_in < 0.0){
        return 0.0;
    }
    else if(s_in > 1.0){
        return 1.0;
    }else{
        return s_in;
    }

}


void SixDimVec::convertToQuat(){

	Eigen::Vector3d aa;

	pos_out[0] = temp->output[0];
	pos_out[1] = temp->output[1];
	pos_out[2] = temp->output[2];
	
	aa[0] = temp->output[3];
	aa[1] = temp->output[4];
	aa[2] = temp->output[5];

	double angle;
	Eigen::Vector3d axis;

	angle = atan2(sin(aa.norm()), cos(aa.norm()));
	axis = aa.normalized();

	quat_out = Eigen::AngleAxisd(angle, axis);

}
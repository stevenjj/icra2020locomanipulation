#include <avatar_locomanipulation/cubic_interpolation_module/seven_dim_vec.hpp>


SevenDimVec::SevenDimVec(){

}


SevenDimVec::SevenDimVec(const int & waypoint_length, const std::string & yaml_filename){
	N = waypoint_length;
	filename = yaml_filename;

	for(int i=1; i<(N-2); ++i){
		temp = std::shared_ptr<SevenDim>(new SevenDim(i, filename) );
		seven_dim_vec.push_back(temp);
	}

	std::cout << "[SevenDimVec] Created" << std::endl;

	std::cout << "seven_dim_vec.size() " << seven_dim_vec.size() << std::endl;
}


SevenDimVec::~SevenDimVec(){

}


void SevenDimVec::evaluate(const double & s_global){
	// Clamp 0.0 <= s <= 1.0
	double s_ = clamp(s_global);

	for(int i=0; i<(N-2); ++i){
		// DEBUG: 
		// std::cout << "s_ = " << s_ << std::endl;
		// std::cout << "Denom: ((double) (1/N-3)) = " << ( (1/ ((double) (N-3)) ) ) << std::endl;
		// std::cout << "Numerator: (s_ - (i / ((double) (N-3)))) = " << ( s_ - (i / ((double) (N-3))) ) << std::endl;
		// std::cout << "Lower Bound: ( i / ((double) (N)) ) = " << ( i / ((double) (N)) ) << std::endl;
		// std::cout << "Upper Bound: ( ((double) (i+3))/((double) (N-1)) ) = " << ( ((double) (i+3))/((double) (N-1)) ) << std::endl;
	
		// Ensure we use global s to select the proper SevenDim to use
		if( ( i / ((double) (N)) ) <= s_ && s_ < ( ((double) (i+3))/((double) (N-1)) ) ){

			// Use the global s to obtain the local s as used by each OneDim
			// s_local = (1/(smax - smin))*(sg - smin)
			double s_local = ( 1 / ( ( ((double) (i+3))/((double) (N-1)) ) - ( i / ((double) (N)) ) )) * ( s_ - ( i / ((double) (N)) ) );
			
			// Feed evaluate the local s value
			seven_dim_vec[i]->evaluate(s_local);
			break;

			// DEBUG:
			// std::cout << "s_local = " << s_local << std::endl;			
			// std::cout <<"i " << i << std::endl;
		}
	} 
	if (s_ >= 1.0){
		seven_dim_vec[N-3]->evaluate(s_);	
		return;	
	}
}


double SevenDimVec::clamp(const double & s_in){
    if (s_in < 0.0){
        return 0.0;
    }
    else if(s_in > 1.0){
        return 1.0;
    }else{
        return s_in;
    }

}
#ifndef ALM_SIX_DIM_H
#define ALM_SIX_DIM_H

#include <avatar_locomanipulation/cubic_interpolation_module/one_dim.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>


class SixDim{
private:
	std::shared_ptr<OneDim> fx, fy, fz, frx, fry, frz, frw;



public:
	SixDim();
	SixDim(const int & first_waypoint, const std::string & yaml_name);

	~SixDim();

	void evaluate(const double & s_local);

	std::vector<double> output;
};






#endif
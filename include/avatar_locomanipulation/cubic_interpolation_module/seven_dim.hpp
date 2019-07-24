#ifndef ALM_SEVEN_DIM_H
#define ALM_SEVEN_DIM_H

#include <avatar_locomanipulation/cubic_interpolation_module/one_dim.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>


class SevenDim{
private:
	std::shared_ptr<OneDim> fx, fy, fz, frx, fry, frz, frw;

public:
	SevenDim();
	SevenDim(const int & first_waypoint, const std::string & yaml_name);

	~SevenDim();

	void evaluate(const double & s_local);

};






#endif
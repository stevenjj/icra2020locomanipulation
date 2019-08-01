#ifndef ALM_CUBIC_INTERPOLATION_SIX_DIM_H
#define ALM_CUBIC_INTERPOLATION_SIX_DIM_H

#include <avatar_locomanipulation/cubic_interpolation_module/cubic_interpolation_one_dim.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>


class CubicInterpolationSixDim{
private:
	std::shared_ptr<CubicInterpolationOneDim> fx, fy, fz, frx, fry, frz, frw;



public:
	CubicInterpolationSixDim();
	CubicInterpolationSixDim(const int & first_waypoint, const std::string & filename_input);

	~CubicInterpolationSixDim();

	void evaluate(const double & s_local);

	std::vector<double> output;
};






#endif
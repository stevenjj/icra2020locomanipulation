#ifndef ALM_CUBIC_INTERPOLATION_ONE_DIM_H
#define ALM_CUBIC_INTERPOLATION_ONE_DIM_H

#include <iostream>
#include <math.h>
#include <vector>
#include <memory>
#include <Configuration.h>
#include <avatar_locomanipulation/helpers/orientation_utils.hpp>


class CubicInterpolationOneDim{
private:
	// Coeffs
	double a0, a1, a2, a3;
	// Given Waypoints
	double wp1, wp2, wp3, wp4;

public:
	CubicInterpolationOneDim();
	CubicInterpolationOneDim(const std::vector<double> & waypoints);

	~CubicInterpolationOneDim();

	void interpolate();

	double evaluate(const double & s_in);
};












#endif
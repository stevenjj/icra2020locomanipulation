#ifndef ALM_ONE_DIM_H
#define ALM_ONE_DIM_H

#include <iostream>
#include <math.h>
#include <vector>
#include <memory>
#include <Configuration.h>


class OneDim{
private:
	// Coeffs
	double a0, a1, a2, a3;
	// Given Waypoints
	double wp1, wp2, wp3, wp4;

public:
	OneDim();
	OneDim(const std::vector<double> & waypoints);

	~OneDim();

	void interpolate();

	void evaluate(const double & s_in);
};












#endif
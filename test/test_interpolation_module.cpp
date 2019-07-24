#include <iostream>
#include <avatar_locomanipulation/cubic_interpolation_module/seven_dim_vec.hpp>
#include <vector>


int main(int argc, char ** argv){
	std::string filename = "door_trajectory.yaml";
	int N = 10;

	SevenDimVec example(N, filename);

	double s = 0.1;
	example.evaluate(s);
	
}
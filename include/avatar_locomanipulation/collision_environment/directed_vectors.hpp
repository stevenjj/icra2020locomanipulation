#ifndef DIRECTED_VECTORS_H
#define DIRECTED_VECTORS_H



struct DirectedVectors{
public:
	// normalized direction vector
	Eigen::Vector3d direction;
	// magnitude of their distance
	double magnitude;
	// name of link/joint from which vector originates
	std::string from;
	// name of link/joint where vector terminates
	std::string to;
};






#endif
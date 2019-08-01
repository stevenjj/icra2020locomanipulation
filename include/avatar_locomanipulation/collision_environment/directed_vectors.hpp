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
  	// If two links are in collision, rather than near collision, this is set true
  	//  and the safety_distance is set to 0.15. When not in collision, safety distance is the 0.075
  	bool using_worldFramePose;
};






#endif
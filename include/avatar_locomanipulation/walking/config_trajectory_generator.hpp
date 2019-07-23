#ifndef ALM_CONFIG_TRAJECTORY_GENERATOR_H
#define ALM_CONFIG_TRAJECTORY_GENERATOR_H

#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

// This class outputs a trajectory of robot configuration q from
//     - a starting configuration, q, and a sequence of footsteps
// 	   - a desired hand/s pose/s trajectory, starting config, and a sequence of footsteps

class ConfigTrajectoryGenerator{
public:
	ConfigTrajectoryGenerator();
	~ConfigTrajectoryGenerator();
};


#endif
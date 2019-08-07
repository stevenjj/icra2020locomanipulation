#ifndef ALM_FEASIBILITY_DATA_GENERATOR_H
#define ALM_FEASIBILITY_DATA_GENERATOR_H

#include <math.h>
#include <time.h>

#include <Configuration.h>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>
#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>

#include <avatar_locomanipulation/data_types/footstep.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>

#include <avatar_locomanipulation/helpers/orientation_utils.hpp>

class FeasibilityDataGenerator{
public:
	FeasibilityDataGenerator();
	~FeasibilityDataGenerator();

    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);

    // Public Member Variables
	std::shared_ptr<RobotModel> robot_model;
	std::shared_ptr<IKModule> ik_start_config_module;
	std::shared_ptr<ConfigTrajectoryGenerator> ctg;

	std::shared_ptr<Task> left_foot_task;
	std::shared_ptr<Task> right_foot_task;
	std::shared_ptr<Task> pelvis_ori;
	std::shared_ptr<Task> com_task;

	void initialize_modules();

};

#endif
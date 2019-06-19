#ifndef IK_MODULE_H
#define IK_MODULE_H

#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <iostream>

class IKModule{
public:
	IKModule();
	~IKModule();

	ValkyrieModel valkyrie;

	void setInitialConfig(const Eigen::VectorXd & q_config);
 // std::vector<Tasks> task_hierarchy;
	// void addTasktoHierarchy;
	// std::vector<>

	
};

#endif
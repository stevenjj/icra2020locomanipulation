#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

// Constructor
ValRvizTranslator::ValRvizTranslator(){}
// Destructor
ValRvizTranslator::~ValRvizTranslator(){}

void ValRvizTranslator::populate_joint_state_msg(const pinocchio::Model & model,
                                                 const Eigen::VectorXd & q, 
                                                 tf::Transform & world_to_pelvis_transform,
                                                 sensor_msgs::JointState & joint_state_msg){
  // Prepare the joint state message
  sensor_msgs::JointState joint_states;
  std::string joint_name;
  double joint_value = 0.0;  


  // Populate joint states
  for (int k = 2; k < model.njoints; k++){
    joint_states.name.push_back(model.names[k]);
    joint_states.position.push_back(q[getJointId(model, model.names[k])]);   
  }


  // Prepare the world to pelvis transform
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(q[0], q[1], q[2]) ); //x, y, z

  // // Initialize quaternion and set the transform
  tf::Quaternion quat_world_to_pelvis(q[3], q[4], q[5], q[6]); //x, y, z, w
  transform.setRotation(quat_world_to_pelvis);

  // Set Output
  joint_state_msg = joint_states;
  world_to_pelvis_transform = transform;
}

int ValRvizTranslator::getJointId(const pinocchio::Model & model, const std::string & name){
  return 7 + model.getJointId(name) - 2;  
}
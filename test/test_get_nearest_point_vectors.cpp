#include <avatar_locomanipulation/collision_module/collision_class.h>
// Algorithms
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
// Robot Model
#include <avatar_locomanipulation/models/valkyrie_model.hpp>
// PseudoInverse
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>
// Map
#include <map>


int main(int argc, char ** argv){
  // First we define the pos/ori of the frames of interest
  Eigen::Vector3d rfoot_cur_pos;
  Eigen::Vector3d lfoot_cur_pos;
  Eigen::Vector3d rankle_cur_pos;
  Eigen::Vector3d lankle_cur_pos;
  Eigen::Vector3d rknee_cur_pos;
  Eigen::Vector3d lknee_cur_pos;
  Eigen::Vector3d pelvis_cur_pos;
  Eigen::Vector3d rshoulder_cur_pos;
  Eigen::Vector3d lshoulder_cur_pos;
  Eigen::Vector3d relbow_cur_pos;
  Eigen::Vector3d lelbow_cur_pos;
  Eigen::Vector3d rwrist_cur_pos;
  Eigen::Vector3d lwrist_cur_pos;
  Eigen::Vector3d rhand_cur_pos;
  Eigen::Vector3d lhand_cur_pos;  

  // Initialize Robot Model
  ValkyrieModel valkyrie;

  // First we will define the configuration of Val
  Eigen::VectorXd q_start;
  q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());

  double theta = 0;//M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q

  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie.getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;

  std::map<std::string, Eigen::Vector3d> positions;
  positions["rfoot"] = rfoot_cur_pos;
  positions["lfoot"] = lfoot_cur_pos;
  positions["rankle"] = rankle_cur_pos;
  positions["lankle"] = lankle_cur_pos;
  positions["rknee"] = rknee_cur_pos;
  positions["lknee"] = lknee_cur_pos;
  positions["rshoulder"] = rshoulder_cur_pos;
  positions["lshoulder"] = lshoulder_cur_pos;
  positions["relbow"] = relbow_cur_pos;
  positions["lelbow"] = lelbow_cur_pos;
  positions["rwrist"] = rwrist_cur_pos;
  positions["lwrist"] = lwrist_cur_pos;
  positions["rhand"] = rhand_cur_pos;
  positions["lhand"] = lhand_cur_pos;
  positions["pelvis"] = pelvis_cur_pos;

  // Build Collision Objects
  std::shared_ptr<Collision> val_object(new Collision() );
  std::shared_ptr<Collision> cart_object(new Collision() );
  std::shared_ptr<Collision> appended_object(new Collision() );

  val_object->build_valkyrie_model_and_geom();
  cart_object->build_cart_model_and_geom();

  std::cout << "BEFORE positions.find(pelvis)->second: " << positions.find("pelvis")->second << std::endl;

  val_object->get_position_of_joints(q_start, positions);

  std::cout << "AFTER positions.find(pelvis)->second: " << positions.find("pelvis")->second << std::endl;

  

  Eigen::VectorXd cart_config(cart_object->get_nq());
  cart_config[0] = 1.3;  cart_config[1] = 0.0;  cart_config[2] = 0.0;
  double theta1 = 0;//M_PI/4.0;	
  Eigen::AngleAxis<double> bb(theta1, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left	
  Eigen::Quaternion<double> quat_init; quat_init =  bb;
  cart_config[3] = quat_init.x();// 0.0;	
  cart_config[4] = quat_init.y(); //0.0;
  cart_config[5] = quat_init.z(); //sin(theta/2.0);
  cart_config[6] = quat_init.w(); //cos(theta/2.0);

  val_object->set_configuration_vector(q_start);
  cart_object->set_configuration_vector(cart_config);

  val_object->append_models(val_object, cart_object, appended_object);

  Eigen::VectorXd appended_config(appended_object->get_nq());
  appended_config << q_start, cart_config;
  appended_object->set_configuration_vector(appended_config);

  Eigen::Vector3d near_handle_point, near_cart_point, difference1, difference2;
  std::map<std::string, Eigen::Vector3d>::iterator posit;

  // We create vectors from handle to val joints
  appended_object->compute_near_point("pelvis", "handle_link", near_handle_point, appended_config);

  std::cout << "near_handle_point = " << near_handle_point << std::endl;

  // // Create and print magnitude and direction of vectors
  // for(posit = positions.begin(); posit != positions.end(); ++posit)
  // {
  // 	//posit->first
  // 	std::cout << "Between handle_link and " << posit->first <<" :" <<  std::endl;
  // 	difference1 = posit->second - near_handle_point;
  // 	std::cout << "Magnitude of distance = " << difference1.norm() << std::endl;
  // 	std::cout << "Normalized Direction: " << std::endl; 
  // 	std::cout << difference1.normalized() << std::endl; 
  // }

  // We create vectors from cart to val joints
  appended_object->compute_near_point("pelvis", "base_link", near_cart_point, appended_config);

  std::cout << "near_cart_point = " << near_cart_point << std::endl;

  // // Create and print magnitude and direction of vectors
  // for(posit = positions.begin(); posit != positions.end(); ++posit)
  // {
  // 	//posit->first
  // 	std::cout << "Between handle_link and " << posit->first <<" :" <<  std::endl;
  // 	difference2 = posit->second - near_cart_point;
  // 	std::cout << "Magnitude of distance = " << difference2.norm() << std::endl;
  // 	std::cout << "Normalized Direction: " << std::endl; 
  // 	std::cout << difference2.normalized() << std::endl; 
  // }

}



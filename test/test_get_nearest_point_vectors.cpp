#include <avatar_locomanipulation/collision_module/collision_class.h>
// Algorithms
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
// Robot Model
#include <avatar_locomanipulation/models/valkyrie_model.hpp>
// PseudoInverse
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>

void get_position(Eigen::VectorXd q_start, Eigen::Vector3d & pelvis_cur_pos, Eigen::Vector3d & rfoot_cur_pos, Eigen::Vector3d & lfoot_cur_pos, Eigen::Vector3d & rankle_cur_pos, Eigen::Vector3d & lankle_cur_pos, Eigen::Vector3d & rknee_cur_pos, Eigen::Vector3d & lknee_cur_pos, Eigen::Vector3d & rshoulder_cur_pos, Eigen::Vector3d & lshoulder_cur_pos, Eigen::Vector3d & relbow_cur_pos, Eigen::Vector3d & lelbow_cur_pos, Eigen::Vector3d & rwrist_cur_pos, Eigen::Vector3d & lwrist_cur_pos, Eigen::Vector3d & rhand_cur_pos, Eigen::Vector3d & lhand_cur_pos);

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

  double theta = M_PI/4.0;
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

  get_position(q_start, pelvis_cur_pos, rfoot_cur_pos, lfoot_cur_pos, rankle_cur_pos, lankle_cur_pos, rknee_cur_pos, lknee_cur_pos, rshoulder_cur_pos, lshoulder_cur_pos, relbow_cur_pos, lelbow_cur_pos, rwrist_cur_pos, lwrist_cur_pos, rhand_cur_pos, lhand_cur_pos);

  // Build Collision Objects
  std::shared_ptr<Collision> val_object(new Collision() );
  std::shared_ptr<Collision> cart_object(new Collision() );
  std::shared_ptr<Collision> appended_object(new Collision() );

  val_object->build_valkyrie_model_and_geom();
  cart_object->build_cart_model_and_geom();

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

}


void get_position(Eigen::VectorXd q_start, Eigen::Vector3d & pelvis_cur_pos, Eigen::Vector3d & rfoot_cur_pos, Eigen::Vector3d & lfoot_cur_pos, Eigen::Vector3d & rankle_cur_pos, Eigen::Vector3d & lankle_cur_pos, Eigen::Vector3d & rknee_cur_pos, Eigen::Vector3d & lknee_cur_pos, Eigen::Vector3d & rshoulder_cur_pos, Eigen::Vector3d & lshoulder_cur_pos, Eigen::Vector3d & relbow_cur_pos, Eigen::Vector3d & lelbow_cur_pos, Eigen::Vector3d & rwrist_cur_pos, Eigen::Vector3d & lwrist_cur_pos, Eigen::Vector3d & rhand_cur_pos, Eigen::Vector3d & lhand_cur_pos)
{

  // Initialize Robot Model
  ValkyrieModel valkyrie;

  // First we define the pos/ori of the frames of interest
  Eigen::Quaternion<double> rfoot_cur_ori;
  Eigen::Quaternion<double> lfoot_cur_ori;
  Eigen::Quaternion<double> rankle_cur_ori;
  Eigen::Quaternion<double> lankle_cur_ori;
  Eigen::Quaternion<double> rknee_cur_ori;
  Eigen::Quaternion<double> lknee_cur_ori;
  Eigen::Quaternion<double> pelvis_cur_ori;
  Eigen::Quaternion<double> rshoulder_cur_ori;
  Eigen::Quaternion<double> lshoulder_cur_ori;
  Eigen::Quaternion<double> relbow_cur_ori;
  Eigen::Quaternion<double> lelbow_cur_ori;
  Eigen::Quaternion<double> rwrist_cur_ori;
  Eigen::Quaternion<double> lwrist_cur_ori;
  Eigen::Quaternion<double> rhand_cur_ori;
  Eigen::Quaternion<double> lhand_cur_ori;

  valkyrie.getFrameWorldPose("pelvis", pelvis_cur_pos, pelvis_cur_ori);
  valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
  valkyrie.getFrameWorldPose("rightCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);
  valkyrie.getFrameWorldPose("leftAnklePitch", rankle_cur_pos, rankle_cur_ori);
  valkyrie.getFrameWorldPose("rightAnklePitch", lankle_cur_pos, lankle_cur_ori);
  valkyrie.getFrameWorldPose("leftKneePitch", rknee_cur_pos, rknee_cur_ori);
  valkyrie.getFrameWorldPose("rightKneePitch", lknee_cur_pos, lknee_cur_ori);
  valkyrie.getFrameWorldPose("rightShoulderRoll", rshoulder_cur_pos, rshoulder_cur_ori);
  valkyrie.getFrameWorldPose("leftShoulderRoll", lshoulder_cur_pos, lshoulder_cur_ori);
  valkyrie.getFrameWorldPose("rightElbowPitch", relbow_cur_pos, relbow_cur_ori);
  valkyrie.getFrameWorldPose("leftElbowPitch", lelbow_cur_pos, lelbow_cur_ori);
  valkyrie.getFrameWorldPose("rightWristRoll", rwrist_cur_pos, rwrist_cur_ori);
  valkyrie.getFrameWorldPose("leftWristRoll", lwrist_cur_pos, lwrist_cur_ori);
  valkyrie.getFrameWorldPose("rightPalm", rhand_cur_pos, rhand_cur_ori);
  valkyrie.getFrameWorldPose("leftPalm", lhand_cur_pos, lhand_cur_ori);
}
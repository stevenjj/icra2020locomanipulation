#include <avatar_locomanipulation/models/valkyrie_model.hpp> 

ValkyrieModel::ValkyrieModel(){
  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf";
  buildPinocchioModel(filename);
  commonInitialization();
}

ValkyrieModel::ValkyrieModel(const std::string & filename){
  buildPinocchioModel(filename);
  commonInitialization();
}

ValkyrieModel::~ValkyrieModel(){
}

void ValkyrieModel::buildPinocchioModel(const std::string & filename){
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
}

void ValkyrieModel::commonInitialization(){
  data = std::unique_ptr<pinocchio::Data>(new pinocchio::Data(model));
  A = Eigen::MatrixXd::Zero(model.nv, model.nv);
  Ainv = Eigen::MatrixXd::Zero(model.nv, model.nv);
  C = Eigen::MatrixXd::Zero(model.nv, model.nv);
  g = Eigen::VectorXd::Zero(model.nv);	
  
  x_com.setZero();
  xdot_com.setZero();
  xddot_com.setZero();

  J_com = Eigen::Matrix3Xd::Zero(3, model.nv);
  Jdot_com = Eigen::Matrix3Xd::Zero(3, model.nv);

  std::cout << "Valkyrie Model Constructed" << std::endl;
}

void ValkyrieModel::updateFullKinematics(const Eigen::VectorXd & q_update){
  // Perform initial forward kinematics
  pinocchio::forwardKinematics(model, *data, q_update);
  // Compute Joint Jacobians
  pinocchio::computeJointJacobians(model,*data, q_update);
  // Update Frame Placements
  pinocchio::updateFramePlacements(model, *data);	
}

void ValkyrieModel::updateKinematicsDerivatives(const Eigen::VectorXd & q_update, const Eigen::VectorXd & qdot_update, const Eigen::VectorXd & qddot_update){
  // Compute the derivatives of the kinematics
  pinocchio::computeForwardKinematicsDerivatives(model, *data, q_update, qdot_update, qddot_update);    
}

void ValkyrieModel::updateJointJacobiansDerivatives(const Eigen::VectorXd & q_update, const Eigen::VectorXd & qdot_update){
  // Computes the time variation of jacobians
  pinocchio::computeJointJacobiansTimeVariation(model, *data, q_update, qdot_update);
}

void ValkyrieModel::get6DTaskJacobian(const std::string & frame_name, Eigen::MatrixXd & J_out){
  tmp_frame_index = model.getFrameId(frame_name);
  pinocchio::getFrameJacobian(model, *data, tmp_frame_index, pinocchio::WORLD, J_out);
}

void ValkyrieModel::get6DTaskJacobianDot(const std::string & frame_name, Eigen::MatrixXd & Jdot_out){
  pinocchio::getFrameJacobianTimeVariation(model, *data, model.getFrameId(frame_name), pinocchio::WORLD, Jdot_out);
}

void ValkyrieModel::getFrameWorldPose(const std::string & name, Eigen::Vector3d & pos, Eigen::Quaternion<double> & ori){
  // Gets the frame index
  tmp_frame_index = model.getFrameId(name);
  // Return Data
  pos = data->oMf[tmp_frame_index].translation();
  ori = data->oMf[tmp_frame_index].rotation();
}


int ValkyrieModel::getDimQ(){
  return model.nq;
}

int ValkyrieModel::getDimQdot(){
  return model.nv;
}

int ValkyrieModel::getJointIndex(const std::string & name){
  return VAL_MODEL_NUM_FLOATING_JOINTS + model.getJointId(name) - VAL_MODEL_JOINT_INDX_OFFSET;  
}

int ValkyrieModel::getJointIndexNoFloatingJoints(const std::string & name){
  return model.getJointId(name) - VAL_MODEL_JOINT_INDX_OFFSET;    
}

void ValkyrieModel::forwardIntegrate(const Eigen::VectorXd & q_start, const Eigen::VectorXd & qdotDt, Eigen::VectorXd & q_post){
  q_post = pinocchio::integrate(model, q_start, qdotDt); // This performs a tangent space integration. Automatically resolves the quaternion components
}

void ValkyrieModel::computeInertiaMatrix(const Eigen::VectorXd & q){
  // Composite Rigid Body Algorithm
  pinocchio::crba(model,*data, q); // only computes the upper triangle part.
  // Copies upper left matrix to lower left
  data->M.triangularView<Eigen::StrictlyLower>() = data->M.transpose().triangularView<Eigen::StrictlyLower>();
  this->A = data->M;
}

void ValkyrieModel::computeInertiaMatrixInverse(const Eigen::VectorXd & q){
  pinocchio::computeMinverse(model,*data,q); // only computes the upper triangle part
  data->Minv.triangularView<Eigen::StrictlyLower>() = data->Minv.transpose().triangularView<Eigen::StrictlyLower>();
  this->Ainv = data->Minv;
}

void ValkyrieModel::computeCoriolisMatrix(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot){
  pinocchio::computeCoriolisMatrix(model, *data, q, qdot);
  this->C = data->C;
}

void ValkyrieModel::computeGravityVector(const Eigen::VectorXd & q){
  pinocchio::computeGeneralizedGravity(model, *data, q);
  this->g = data->g;
}


void ValkyrieModel::computeCoMPos(const Eigen::MatrixBase<Eigen::VectorXd> & q){
  this->x_com = pinocchio::centerOfMass(model, *data, q);  
}

void ValkyrieModel::computeCoMPosVel(const Eigen::MatrixBase<Eigen::VectorXd> & q, const Eigen::MatrixBase<Eigen::VectorXd> & qdot){
  this->x_com = pinocchio::centerOfMass(model, *data, q, qdot);
  this->xdot_com = data->vcom[0]; 
}

void ValkyrieModel::computeCoMPosVelAcc(const Eigen::MatrixBase<Eigen::VectorXd> & q, const Eigen::MatrixBase<Eigen::VectorXd> & qdot, const Eigen::MatrixBase<Eigen::VectorXd> & qddot){
  this->x_com = pinocchio::centerOfMass(model, *data, q, qdot, qddot);
  this->xdot_com = data->vcom[0];   
  this->xddot_com = data->acom[0];     
}

void ValkyrieModel::computeCoMJacobian(){
  J_com = pinocchio::jacobianCenterOfMass(model, *data);
}

void ValkyrieModel::computeCoMJacobian(const Eigen::MatrixBase<Eigen::VectorXd> & q){
  J_com = pinocchio::jacobianCenterOfMass(model, *data, q);
}

void ValkyrieModel::computeCoMJacobianDot(const Eigen::MatrixBase<Eigen::VectorXd> & q, const Eigen::MatrixBase<Eigen::VectorXd> & qdot){
  // To compute Jdot_com, we need to have called computeForwardKinematicsDerivatives and centerOfMass computation with position and velocity.
  pinocchio::centerOfMass(model, *data, q, qdot);
  pinocchio::getCenterOfMassVelocityDerivatives(model, *data, Jdot_com);
}


void ValkyrieModel::printJointNames(){
  // List joint names:
  for (int k=0 ; k<model.njoints ; ++k){
    std::cout << "pinocchio id:" << k << " " << model.names[k] <<  std::endl;
  }	
}

void ValkyrieModel::printFrameNames(){
  // List frames:
  std::cout << "number of frames in the model:" << model.frames.size() << std::endl;
  for (int k=0 ; k<model.frames.size() ; ++k){
    std::cout << "frame:" << k << " " << model.frames[k].name <<  std::endl;
  }	
}
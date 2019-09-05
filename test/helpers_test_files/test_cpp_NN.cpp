#include <avatar_locomanipulation/helpers/NeuralNetModel.hpp>

# include <cstdlib>
# include <iostream>
# include <iomanip>
# include <cmath>

#include <avatar_locomanipulation/helpers/IOUtilities.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <avatar_locomanipulation/BinaryClassifierQuery.h>

#include <avatar_locomanipulation/models/robot_model.hpp>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

#define CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE 0
#define CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE 1

#define CONTACT_TRANSITION_DATA_LEFT_HAND 0
#define CONTACT_TRANSITION_DATA_RIGHT_HAND 1
#define CONTACT_TRANSITION_DATA_BOTH_HANDS 2

Eigen::Vector3d quatToVec(const Eigen::Quaterniond & ori){
	Eigen::AngleAxisd tmp_ori(ori.normalized()); // gets the normalized version of ori and sets it to an angle axis representation
  Eigen::Vector3d ori_vec = tmp_ori.axis()*tmp_ori.angle();
  return ori_vec;
}

void normalizeInputCalculate(const Eigen::VectorXd & x_in, const Eigen::VectorXd & data_mean, const Eigen::VectorXd & data_std_dev, Eigen::VectorXd & x_normalized) {
  // std::cout << x_in.rows() << " " << data_mean.rows() << " " << data_std_dev.rows() << std::endl;
  x_normalized = (x_in - data_mean).cwiseQuotient(data_std_dev);
}

int main(int argc, char ** argv){
  ParamHandler param_handler;
  std::string model_path = "/home/stevenjj/nasa_ws/src/avatar_locomanipulation/nn_models/cpp_models/model_baseline_11500pts.yaml";

  std::cout << "Loading Model..." << std::endl;
  myYAML::Node model = myYAML::LoadFile(model_path);
  NeuralNetModel nn_transition(model, false);
  std::cout << "Loaded" << std::endl;

  double stance_origin = CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE;
  double manipulation_type = CONTACT_TRANSITION_DATA_RIGHT_HAND;

  Eigen::Vector3d swing_foot_start_pos(0.15411,-0.2074,0.0);
  Eigen::Quaterniond swing_foot_start_ori(0.998433,0,0,-0.0559473);

  Eigen::Vector3d pelvis_pos(-0.04466, -0.0581, 0.9522);
  Eigen::Quaterniond pelvis_ori(0.999608,0,0,-0.0279);

  Eigen::Vector3d landing_foot_pos(0.08006, -0.3344725,0);
  Eigen::Quaterniond landing_foot_ori(0.9746387,0,0,-0.22378);

  Eigen::Vector3d right_hand_start_pos(0.37345209, -0.67836299, 1.0333);
  Eigen::Quaterniond right_hand_start_ori(0.778100, 0.22986, -0.234809 ,0.535339);

  Eigen::Vector3d left_hand_start_pos(0,0,0);
  Eigen::Quaterniond left_hand_start_ori(1,0,0,0);

  Eigen::VectorXd rawDatum(32);
  Eigen::VectorXd datum(32);
	Eigen::MatrixXd data(5,32);

  rawDatum << stance_origin, manipulation_type,
    swing_foot_start_pos, quatToVec(swing_foot_start_ori),
    pelvis_pos, quatToVec(pelvis_ori),
    landing_foot_pos, quatToVec(landing_foot_ori),
    right_hand_start_pos, quatToVec(right_hand_start_ori),
    left_hand_start_pos, quatToVec(left_hand_start_ori);

  //Normalization Params
  param_handler.load_yaml_file("/home/stevenjj/nasa_ws/src/avatar_locomanipulation/nn_models/cpp_models/normalization_params.yaml");
  std::vector<double> vmean;
  param_handler.getVector("x_train_mean", vmean);
  std::vector<double> vstd_dev;
  param_handler.getVector("x_train_std", vstd_dev);

  Eigen::VectorXd mean(32);
  Eigen::VectorXd std_dev(32);
  for (int ii = 0; ii < 32; ii++){
    mean[ii] = vmean[ii];
    std_dev[ii] = vstd_dev[ii];
  }

  normalizeInputCalculate(rawDatum, mean, std_dev, datum);
	data.row(0) = datum;
	data.row(1) = datum;
	data.row(2) = datum;
	data.row(3) = datum;
	data.row(4) = datum;
	// std::cout << "datum: " << datum << std::endl;
  Eigen::MatrixXd pred(5,1);
  pred = nn_transition.GetOutput(data);
  std::cout << "Prediction: " << pred << std::endl;

  // Test Inference speed:
  auto t1 = Clock::now();
  auto t2 = Clock::now();
  double time_span;

  Eigen::MatrixXd x_vec(1,32);
  Eigen::MatrixXd y_vec(1,1);
  x_vec.row(0) = datum;


  while(true){
    t1 = Clock::now();
    y_vec = nn_transition.GetOutput(x_vec);
    t2 = Clock::now();
    time_span = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();

    std::cout << " 1 pred = " << y_vec << std::endl;
    std::cout << " inference time = " << time_span << " seconds" << std::endl;
    std::cout << " freq = " << (1.0/time_span) << " Hz" << std::endl;
    std::cout << " " << std::endl;

    t1 = Clock::now();    
    pred = nn_transition.GetOutput(data);
    t2 = Clock::now();

    time_span = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();
    std::cout << " 5 pred = " << pred.transpose() << std::endl;
    std::cout << " inference time = " << time_span << " seconds" << std::endl;
    std::cout << " freq = " << (1.0/time_span) << " Hz" << std::endl;
    std::cout << " " << std::endl;

  }


  return 0;
}

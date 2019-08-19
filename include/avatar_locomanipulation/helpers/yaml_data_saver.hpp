#ifndef ALM_YAML_DATA_SAVER
#define ALM_YAML_DATA_SAVER

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <stdlib.h>
#include <iostream>

namespace data_saver{
  void emit_value(YAML::Emitter & out, const std::string & key, const double & value);
  void emit_integer(YAML::Emitter & out, const std::string & key, const int & value);
  void emit_string(YAML::Emitter & out, const std::string & key, const std::string & value);
  void emit_position(YAML::Emitter & out, const std::string & key, const Eigen::Vector3d & pos);
  void emit_orientation(YAML::Emitter & out, const std::string & key, const Eigen::Quaterniond & quat);
  void emit_joint_configuration(YAML::Emitter & out, const std::string & key, const Eigen::VectorXd & q_in);
  void emit_orientation_vector(YAML::Emitter & out, const std::string & key, const Eigen::Vector3d & ori);
  void emit_gmm_sigma(YAML::Emitter & out, const std::string & key, const Eigen::MatrixXd & sigma);
  void emit_gmm_mu(YAML::Emitter & out, const std::string & key, const Eigen::VectorXd & mu);
  void emit_gmm_normalize(YAML::Emitter & out, const Eigen::VectorXd & mu, const Eigen::VectorXd & sigma, const std::vector<std::string> & vars);
}

#endif

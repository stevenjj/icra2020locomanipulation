#ifndef ALM_YAML_DATA_SAVER
#define ALM_YAML_DATA_SAVER

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace data_saver{
  void emit_position(YAML::Emitter & out, const std::string & key, const Eigen::Vector3d & pos);
  void emit_orientation(YAML::Emitter & out, const std::string & key, const Eigen::Quaterniond & quat);
  void emit_joint_configuration(YAML::Emitter & out, const std::string & key, const Eigen::VectorXd & q_in);
  void emit_orientation_vector(YAML::Emitter & out, const std::string & key, const Eigen::Vector3d & ori);
  void emit_gmm_sigma(YAML::Emitter & out, const std::string & key, const Eigen::MatrixXd & sigma);
  void emit_gmm_mu(YAML::Emitter & out, const std::string & key, const Eigen::VectorXd & mu);
}

#endif
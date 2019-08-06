#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>

namespace data_saver{

  void emit_value(YAML::Emitter & out, const std::string & key, const double & value) {
    out << YAML::Key << key;
    out << YAML::Value << value;
  }

  void emit_string(YAML::Emitter & out, const std::string & key, const std::string & value) {
    out << YAML::Key << key;
    out << YAML::Value << value;
  }

  void emit_position(YAML::Emitter & out, const std::string & key, const Eigen::Vector3d & pos){
    out << YAML::Key << key;
    out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << pos[0];
      out << YAML::Key << "y" << YAML::Value << pos[1];
      out << YAML::Key << "z" << YAML::Value << pos[2];
      out << YAML::EndMap;
  }

  void emit_orientation(YAML::Emitter & out, const std::string & key, const Eigen::Quaterniond & quat){
    out << YAML::Key << key;
    out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << quat.x();
      out << YAML::Key << "y" << YAML::Value << quat.y();
      out << YAML::Key << "z" << YAML::Value << quat.z();
      out << YAML::Key << "w" << YAML::Value << quat.w();
      out << YAML::EndMap;
  }


  void emit_joint_configuration(YAML::Emitter & out, const std::string & key, const Eigen::VectorXd & q_in){
    out << YAML::Key << key;
    out << YAML::Value;
      out << YAML::Flow;
      out << YAML::BeginSeq;
      for (size_t i = 0; i < q_in.size(); i++){
        out << q_in[i] ;
      }
      out << YAML::EndSeq;
  }

  void emit_orientation_vector(YAML::Emitter & out, const std::string & key, const Eigen::Vector3d & ori){
    out << YAML::Key << key;
    out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "rx" << YAML::Value << ori[0];
      out << YAML::Key << "ry" << YAML::Value << ori[1];
      out << YAML::Key << "rz" << YAML::Value << ori[2];
      out << YAML::EndMap;
  }

  void emit_gmm_sigma(YAML::Emitter & out, const std::string & key, const Eigen::MatrixXd & sigma){
    out << YAML::Key << key;
    out << YAML::Value;
      out << YAML::Flow;
      out << YAML::BeginSeq;
      for (size_t k = 0; k < sigma.rows(); k++){
      for (size_t i = 0; i < sigma.cols(); i++){
        out << sigma(i,k);
      }
    }
      out << YAML::EndSeq;

  }

  void emit_gmm_normalize(YAML::Emitter & out, const Eigen::VectorXd & mu, const Eigen::VectorXd & sigma, const std::vector<std::string> & vars){
    if (mu.size() == sigma.size() && mu.size() == vars.size()) {
      // std::cout << mu << std::endl;
      // std::cout << sigma << std::endl;
      out << YAML::Key << "Normalization Data";
      out << YAML::Value;
        out << YAML::BeginMap;
        for (size_t i = 0; i < mu.size(); i++){
          out << YAML::Key << vars[i];
          out << YAML::Value;
            out << YAML::BeginMap;
            out << YAML::Key << "Mu";
                out << YAML::Value << mu[i];
            out << YAML::Key << "Sigma";
                out << YAML::Value << sigma[i];
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
      } else {
        std::cout << "How do you expect me to work if you don't give me proper directions?" << std::endl;
      }
  }

}

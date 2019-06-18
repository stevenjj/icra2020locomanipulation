#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>

namespace data_saver{

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

}

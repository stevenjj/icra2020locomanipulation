#include <Configuration.h>
#include <avatar_locomanipulation/helpers/gmm_fit.hpp>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <avatar_locomanipulation/feasibility/feasibility.hpp>
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

#include <avatar_locomanipulation/helpers/orientation_utils.hpp>
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>
#include <iostream>
#include <fstream>

int main(int argc, char ** argv){
  ValkyrieModel valkyrie;
  GMMFit gmmfitter;
  feasibility fiz;
  ParamHandler param_handler;

  Eigen::Vector3d lhand_cur_pos;
  Eigen::Quaternion<double> lhand_cur_ori;
  Eigen::Vector3d lhand_cur_ori_vec;

  Eigen::VectorXd datum;
  datum = Eigen::VectorXd::Zero(6);

  std::vector<Eigen::VectorXd> list_of_datums;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  std::string filename;
  for(int i=0; i<1000; i++){
    fiz.generateRandomLeftArm();
    lhand_cur_pos=fiz.lhand_cur_pos;
    lhand_cur_ori=fiz.lhand_cur_ori;

    math_utils::convert(lhand_cur_ori, lhand_cur_ori_vec);

    YAML::Emitter out;
    out<< YAML::BeginMap;
      data_saver::emit_position(out,"LPalmPos",lhand_cur_pos);
      data_saver::emit_orientation_vector(out,"LPalmOri",lhand_cur_ori_vec);
    out << YAML::EndMap;
    filename =  std::string(THIS_PACKAGE_PATH) + "../lMD/h" + std::to_string(i) + ".yaml";
    std::ofstream file_output_stream(filename);
    file_output_stream << out.c_str();
  }
}

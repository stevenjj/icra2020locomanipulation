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

  Eigen::Vector3d rhand_cur_pos;
  Eigen::Quaternion<double> rhand_cur_ori;
  Eigen::AngleAxis<double> rhand_cur_ori_aa;
  Eigen::Vector3d rhand_cur_ori_vec;

for(int i=0; i<2; i++){
  fiz.generateRandomArm();
  rhand_cur_pos=fiz.rhand_cur_pos;
  rhand_cur_ori=fiz.rhand_cur_ori;

  std::cout << "x:" << rhand_cur_ori.x()
            << "y:" << rhand_cur_ori.y()
            << "z:" << rhand_cur_ori.z()
            << "w:" << rhand_cur_ori.w() << std::endl;
  rhand_cur_ori_aa = rhand_cur_ori;
  rhand_cur_ori_vec = rhand_cur_ori_aa.axis()*rhand_cur_ori_aa.angle();

  std::cout << "before: " << std::endl;
  std::cout << "  angle " << rhand_cur_ori_aa.angle() << std::endl;
  std::cout << "  axis " << rhand_cur_ori_aa.axis().transpose() << std::endl;

  math_utils::convert(rhand_cur_ori, rhand_cur_ori_vec);

  std::cout << "after: " << std::endl;
  std::cout << "  angle " << rhand_cur_ori_vec.norm() << std::endl;
  std::cout << "  axis " << (rhand_cur_ori_vec / rhand_cur_ori_vec.norm()).transpose() << std::endl;


  // YAML::Emitter out;
  // out<< YAML::BeginMap;
  //   data_saver::emit_position(out,"RPalmPos",rhand_cur_pos);
  //   data_saver::emit_orientation(out,"RPalmOri",rhand_cur_ori);
  // out << YAML::EndMap;
  // char filename[64];
  // // filename = "file.yaml"
  // snprintf(filename, sizeof(char)*32, "RPalmLoc%d.yaml", i);
  // std::ofstream file_output_stream(filename);
  // file_output_stream << out.c_str();
}
return 0;

}

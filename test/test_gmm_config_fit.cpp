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

  int task;

  Eigen::Vector3d rhand_cur_pos;
  Eigen::Quaternion<double> rhand_cur_ori;
  Eigen::Vector3d rhand_cur_ori_vec;

  Eigen::VectorXd datum;
  std::vector<Eigen::VectorXd> list_of_datums;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  task = 2;
if (task == 1){
  for(int i=0; i<100000; i++){
    fiz.generateRandomArm();
    rhand_cur_pos=fiz.rhand_cur_pos;
    rhand_cur_ori=fiz.rhand_cur_ori;

    // std::cout << "x:" << rhand_cur_ori.x()
    //           << "y:" << rhand_cur_ori.y()
    //           << "z:" << rhand_cur_ori.z()
    //           << "w:" << rhand_cur_ori.w() << std::endl;
    // rhand_cur_ori_aa = rhand_cur_ori;
    // rhand_cur_ori_vec = rhand_cur_ori_aa.axis()*rhand_cur_ori_aa.angle();

    // std::cout << "before: " << std::endl;
    // std::cout << "  angle " << rhand_cur_ori_aa.angle() << std::endl;
    // std::cout << "  axis " << rhand_cur_ori_aa.axis().transpose() << std::endl;

    math_utils::convert(rhand_cur_ori, rhand_cur_ori_vec);

    // std::cout << "after: " << std::endl;
    // std::cout << "  angle " << rhand_cur_ori_vec.norm() << std::endl;
    // std::cout << "  axis " << (rhand_cur_ori_vec / rhand_cur_ori_vec.norm()).transpose() << std::endl;

  //  std::string filename = "THIS_PACKAGE_PATH../../some_folder"

    YAML::Emitter out;
    out<< YAML::BeginMap;
      data_saver::emit_position(out,"RPalmPos",rhand_cur_pos);
      data_saver::emit_orientation_vector(out,"RPalmOri",rhand_cur_ori_vec);
    out << YAML::EndMap;
    char filename[64];
    // filename = "file.yaml"
    snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/h%d.yaml", i);
    std::ofstream file_output_stream(filename);
    file_output_stream << out.c_str();
  }
} else if (task == 2){
  datum = Eigen::VectorXd::Zero(6);
  gmmfitter.setNumClusters(5);
  gmmfitter.setDim(6);
  gmmfitter.initializeNormalization();
  for(int i=0; i<4000; i++){
    char filename[64];
    snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/h%d.yaml", i);
    param_handler.load_yaml_file(filename);

    param_handler.getNestedValue({"RPalmPos", "x"}, x);
    param_handler.getNestedValue({"RPalmPos", "y"}, y);
    param_handler.getNestedValue({"RPalmPos", "z"}, z);

    param_handler.getNestedValue({"RPalmOri", "rx"}, rx);
    param_handler.getNestedValue({"RPalmOri", "ry"}, ry);
    param_handler.getNestedValue({"RPalmOri", "rz"}, rz);

    datum[0] = x;
    datum[1] = y;
    datum[2] = z;
    datum[3] = rx;
    datum[4] = ry;
    datum[5] = rz;

    gmmfitter.addData(datum);
  }
  gmmfitter.prepData();
  gmmfitter.randInitialGuess();
  gmmfitter.expectationMax();
}
return 0;

}

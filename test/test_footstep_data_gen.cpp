// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>

#include <avatar_locomanipulation/feasibility/feasibility.hpp>

#include <avatar_locomanipulation/helpers/orientation_utils.hpp>
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

int main(int argc, char **argv){

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

  RobotModel valkyrie(filename, meshDir, srdf_filename);
  int good_data = 0;
  int bad_data = 0;
  int good_des_data = 1000;
  int bad_des_data = 0;
  Eigen::VectorXd  q_start(valkyrie.getDimQ()); q_start.setZero();
  Eigen::VectorXd  q_end(valkyrie.getDimQ()); q_end.setZero();
  Eigen::VectorXd  q_data(valkyrie.getDimQ()); q_data.setZero();
  double error;
  feasibility	fiz;

  Eigen::Vector3d rfoot_cur_pos;
  Eigen::Quaternion<double> rfoot_cur_ori;
  Eigen::Vector3d lfoot_cur_pos;
  Eigen::Quaternion<double> lfoot_cur_ori;

  Eigen::Vector3d lhand_cur_ori_vec;
  Eigen::Vector3d rhand_cur_ori_vec;
  Eigen::Vector3d lfoot_cur_ori_vec;

  Eigen::Vector3d rhand_cur_pos;
  Eigen::Quaternion<double> rhand_cur_ori;
  Eigen::Vector3d lhand_cur_pos;
  Eigen::Quaternion<double> lhand_cur_ori;

  Eigen::Vector3d lfoot_des_pos;

  Eigen::Vector3d com_des_pos;
  Eigen::Vector3d com_cur_pos;
  Eigen::Vector3d pel_cur_pos;
  Eigen::Quaternion<double> pelvis_des_ori;

  while (good_data<good_des_data) {
    fiz.initialize_configuration_top();
    fiz.initialize_desired_top();
    fiz.InverseKinematicsTop(300);

    q_start = fiz.q_start;
    q_end = fiz.q_end;
    q_data = fiz.q_data;
    error = fiz.ik_error_norm;

    rfoot_cur_pos=fiz.rfoot_cur_pos;
    rfoot_cur_ori=fiz.rfoot_cur_ori;
    lfoot_cur_pos=fiz.lfoot_cur_pos;
    lfoot_cur_ori=fiz.lfoot_cur_ori;

    lfoot_des_pos=fiz.lfoot_des_pos;

    com_cur_pos = fiz.com_cur_pos;
    pel_cur_pos = fiz.pelvis_cur_pos;

    com_des_pos=fiz.com_des_pos;
    pelvis_des_ori=fiz.pelvis_des_quat;

    fiz.valkyrie.getFrameWorldPose("rightPalm", rhand_cur_pos, rhand_cur_ori);
    fiz.valkyrie.getFrameWorldPose("leftPalm", lhand_cur_pos, lhand_cur_ori);

    math_utils::convert(lhand_cur_ori, lhand_cur_ori_vec);
    math_utils::convert(rhand_cur_ori, rhand_cur_ori_vec);
    math_utils::convert(lfoot_cur_ori, lfoot_cur_ori_vec);


    if (error > 1e-6) {
      YAML::Emitter out;
      out<< YAML::BeginMap;
        data_saver::emit_position(out,"LPalmPos",lhand_cur_pos);
        data_saver::emit_orientation_vector(out,"LPalmOri",lhand_cur_ori_vec);
        data_saver::emit_position(out,"RPalmPos",rhand_cur_pos);
        data_saver::emit_orientation_vector(out,"RPalmOri",rhand_cur_ori_vec);

        data_saver::emit_position(out,"LFootPos",lfoot_cur_pos);
        data_saver::emit_orientation_vector(out,"LFootOri",lfoot_cur_ori_vec);
        data_saver::emit_position(out, "CoM", com_cur_pos);
        data_saver::emit_position(out, "PelvisPos", pel_cur_pos);

        data_saver::emit_string(out,"LFootDesX","-.15 to .35");
        data_saver::emit_string(out,"LFootDesY",".2 to .4");
        data_saver::emit_string(out,"LFootDesZ","0.0 to .15");
        data_saver::emit_string(out,"LFootYaw","-.15 to .6");
        data_saver::emit_string(out,"PelDesHeight", "1.03 to 1.1");
      out << YAML::EndMap;
      char filename[64];
      // filename = "file.yaml"
      snprintf(filename, sizeof(char)*64, "/home/mihir/lMD/1/bad/l%d.yaml", bad_data);
      std::ofstream file_output_stream(filename);
      file_output_stream << out.c_str();
      bad_data++;
    } else if (error < 1e-6) {
      YAML::Emitter out;
      out<< YAML::BeginMap;
        data_saver::emit_position(out,"LPalmPos",lhand_cur_pos);
        data_saver::emit_orientation_vector(out,"LPalmOri",lhand_cur_ori_vec);
        data_saver::emit_position(out,"RPalmPos",rhand_cur_pos);
        data_saver::emit_orientation_vector(out,"RPalmOri",rhand_cur_ori_vec);

        data_saver::emit_position(out,"LFootPos",lfoot_cur_pos);
        data_saver::emit_orientation_vector(out,"LFootOri",lfoot_cur_ori_vec);
        data_saver::emit_position(out, "CoM", com_cur_pos);
        data_saver::emit_position(out, "PelvisPos", pel_cur_pos);

        data_saver::emit_string(out,"LFootDesX","-.15 to .35");
        data_saver::emit_string(out,"LFootDesY",".2 to .4");
        data_saver::emit_string(out,"LFootDesZ","0.0 to .15");
        data_saver::emit_string(out,"LFootYaw","-.15 to .6");
        data_saver::emit_string(out,"PelDesHeight", "1.03 to 1.1");
      out << YAML::EndMap;
      char filename[64];
      // filename = "file.yaml"
      snprintf(filename, sizeof(char)*64, "/home/mihir/lMD/1/good/l%d.yaml", good_data);
      std::ofstream file_output_stream(filename);
      file_output_stream << out.c_str();
      good_data++;
    }
  }
}

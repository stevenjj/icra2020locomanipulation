// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <avatar_locomanipulation/models/robot_model.hpp>

#include <iostream>
#include <fstream>

#include <math.h>


void emit_7dof(YAML::Emitter & out, std::string & waypoint_name, std::vector<double> & values){
	out << YAML::Key << waypoint_name;
    out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << values[0];
      out << YAML::Key << "y" << YAML::Value << values[1];
      out << YAML::Key << "z" << YAML::Value << values[2];
      out << YAML::Key << "rx" << YAML::Value << values[3];
      out << YAML::Key << "ry" << YAML::Value << values[4];
      out << YAML::Key << "rz" << YAML::Value << values[5];
      out << YAML::Key << "rw" << YAML::Value << values[6];
      out << YAML::EndMap;
}



int main(int argc, char **argv){
  ParamHandler param_handler;
  double radius, rotation_angle, linear_length, pose_spacing, hand_distance;
  Eigen::Vector3d fixed_frame_pos;
  Eigen::Quaternion<double> q_r_init, q_l_init;

  param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/cart_parameters.yaml");

  param_handler.getValue("radius", radius);

  param_handler.getValue("rotation_angle", rotation_angle);

  param_handler.getValue("linear_length", linear_length);

  param_handler.getValue("pose_spacing", pose_spacing);

  param_handler.getValue("hand_distance", hand_distance);

  param_handler.getNestedValue({"fixed_frame_position", "x"}, fixed_frame_pos[0]);
  param_handler.getNestedValue({"fixed_frame_position", "y"}, fixed_frame_pos[1]);
  param_handler.getNestedValue({"fixed_frame_position", "z"}, fixed_frame_pos[2]);


  double rx, ry, rz, rw;

  param_handler.getNestedValue({"right_hand_initial_orientation", "x"}, rx);
  param_handler.getNestedValue({"right_hand_initial_orientation", "y"}, ry);
  param_handler.getNestedValue({"right_hand_initial_orientation", "z"}, rz);
  param_handler.getNestedValue({"right_hand_initial_orientation", "w"}, rw);

  q_r_init.x() = rx; q_r_init.y() = ry; q_r_init.z() = rz; q_r_init.w() = rw;
  q_r_init.normalize();

  param_handler.getNestedValue({"left_hand_initial_orientation", "x"}, rx);
  param_handler.getNestedValue({"left_hand_initial_orientation", "y"}, ry);
  param_handler.getNestedValue({"left_hand_initial_orientation", "z"}, rz);
  param_handler.getNestedValue({"left_hand_initial_orientation", "w"}, rw);

  q_l_init.x() = rx; q_l_init.y() = ry; q_l_init.z() = rz; q_l_init.w() = rw;
  q_l_init.normalize();


  double arc_length = rotation_angle * radius;
  int N = static_cast<int>(arc_length / pose_spacing);// Number of wps to describe the arc

  int M = static_cast<int>(linear_length / pose_spacing); // Number of wps to describe the straight push

  double dtheta, dl;

  dtheta = rotation_angle / static_cast<double>(N);

  dl = linear_length / static_cast<double>(M);

	Eigen::Quaternion<double> q; // The quaternion (0, 0, sin(theta/2), cos(theta/2))
																// to be post multiplied by q_init
	Eigen::Quaternion<double> q_r_i, q_l_i; // q_i = q_init * q
	std::string left_waypoint_string, right_waypoint_string;
	std::vector<double> right_values(7); // {x, y, z, rx, ry, rz, rw}
  std::vector<double> left_values(7); // {x, y, z, rx, ry, rz, rw}

	// Begin our emitter
	YAML::Emitter out;
  out << YAML::BeginMap;


  if(linear_length == 0.0){

  	data_saver::emit_value(out, "num_waypoints", N+1);
  	// Generate the waypoints around the arc
  	for(int i=0; i<=N; ++i){
  		right_waypoint_string = "right_waypoint_" + std::to_string(i+1);
      left_waypoint_string = "left_waypoint_" + std::to_string(i+1);

      // x position
  		left_values[0] = cos(-1.57 + static_cast<double>(i)*dtheta)*radius;
      right_values[0] = cos(-1.57 + static_cast<double>(i)*dtheta)*(radius+hand_distance);
      // y position
      left_values[1] = sin(-1.57 + static_cast<double>(i)*dtheta)*radius;
      right_values[1] = sin(-1.57 + static_cast<double>(i)*dtheta)*(radius+hand_distance);
      // z position
      left_values[2] = 0.;
      right_values[2] = 0.; 

  		// Get the updated quaternions
  		q.x() = 0; q.y() = 0; q.z() = sin((static_cast<double>(i)*dtheta) / 2); q.w() = cos((static_cast<double>(i)*dtheta) / 2);
  		q_l_i = q*q_l_init;
      q_r_i = q*q_r_init;

      left_values[3] = q_l_i.x(); left_values[4] = q_l_i.y(); left_values[5] = q_l_i.z(); left_values[6] = q_l_i.w();
  		right_values[3] = q_r_i.x(); right_values[4] = q_r_i.y(); right_values[5] = q_r_i.z(); right_values[6] = q_r_i.w();

      emit_7dof(out, left_waypoint_string, left_values);
  		emit_7dof(out, right_waypoint_string, right_values);
  	}

  }
    else{
  	// When l!=0 we have 5 additional waypoints for the length
  	data_saver::emit_value(out, "num_waypoints", M+N+2);
  	// Generate waypoints fo the line 
  	for(int i=0; i<=M; ++i){
  		right_waypoint_string = "right_waypoint_" + std::to_string(i+1);
      left_waypoint_string = "left_waypoint_" + std::to_string(i+1);
      // x position
      left_values[0] = i*dl;
      right_values[0] = i*dl;
      // y position
      left_values[1] = radius;
      right_values[1] = radius + hand_distance;
      // z position
      left_values[2] = 0.;
      right_values[2] = 0.; 

      left_values[3] = q_l_init.x(); left_values[4] = q_l_init.y(); left_values[5] = q_l_init.z(); left_values[6] = q_l_init.w();
      right_values[3] = q_r_init.x(); right_values[4] = q_r_init.y(); right_values[5] = q_r_init.z(); right_values[6] = q_r_init.w();

  		emit_7dof(out, left_waypoint_string, left_values);
      emit_7dof(out, right_waypoint_string, right_values);
  	}
  	// Generate waypoints for the arc
  	for(int i=0; i<=N; ++i){
      right_waypoint_string = "right_waypoint_" + std::to_string(i+M+2);
      left_waypoint_string = "left_waypoint_" + std::to_string(i+M+2);

      // x position
      left_values[0] = linear_length + cos(-1.57 + static_cast<double>(i)*dtheta)*radius;
      right_values[0] = linear_length + cos(-1.57 + static_cast<double>(i)*dtheta)*(radius+hand_distance);
      // y position
      left_values[1] = sin(-1.57 + static_cast<double>(i)*dtheta)*radius;
      right_values[1] = sin(-1.57 + static_cast<double>(i)*dtheta)*(radius+hand_distance);
      // z position
      left_values[2] = 0.;
      right_values[2] = 0.; 

      // Get the updated quaternions
      q.x() = 0; q.y() = 0; q.z() = sin((static_cast<double>(i)*dtheta) / 2); q.w() = cos((static_cast<double>(i)*dtheta) / 2);
      q_l_i = q*q_l_init;
      q_r_i = q*q_r_init;

      left_values[3] = q_l_i.x(); left_values[4] = q_l_i.y(); left_values[5] = q_l_i.z(); left_values[6] = q_l_i.w();
      right_values[3] = q_r_i.x(); right_values[4] = q_r_i.y(); right_values[5] = q_r_i.z(); right_values[6] = q_r_i.w();

      emit_7dof(out, left_waypoint_string, left_values);
      emit_7dof(out, right_waypoint_string, right_values);
  	}

  }


  out << YAML::EndMap;
	std::ofstream file_output_stream("cart_hand_trajectory.yaml");
  file_output_stream << out.c_str();
}
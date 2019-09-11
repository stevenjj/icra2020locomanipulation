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

	param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/bag_lift_putdown_parameters.yaml");

	// Get our Configuration parameters
	double r; // radius of arc
	param_handler.getValue("radius", r);

	double theta;
	param_handler.getValue("rotation_angle", theta);

	double l;
	param_handler.getValue("lift_height", l);

	double pose_spacing;
	param_handler.getValue("pose_spacing", pose_spacing);

	double arc_length = theta / r;
	int N = static_cast<int>(arc_length / 0.05);// Number of wps to describe the arc

  	int M = static_cast<int>(l / 0.05); // Number of wps to describe each of the up and down motions
	
  	std::cout << "M = " << M << std::endl;
  	std::cout << "N = " << N << std::endl;

	Eigen::Vector3d hand_init;
	hand_init[0] = 0.0; hand_init[1] = 0.0; hand_init[2] = 0.75;
	Eigen::Quaternion<double> q_init;
	q_init.x() = 0.11781; q_init.y() = 0.39669;
	q_init.z() = 0.60298; q_init.w() = 0.68204;
	q_init.normalize();

	// Helper variables
	double dtheta = theta / static_cast<double>(N);
	double dl = l / 5.;
	Eigen::Quaternion<double> q; // The quaternion (0, 0, sin(theta/2), cos(theta/2))
																// to be post multiplied by q_init
	Eigen::Quaternion<double> q_i; // q_i = q_init * q
	std::string waypoint_string;
	std::vector<double> values(7); // {x, y, z, rx, ry, rz, rw}

	// Begin our emitter
	YAML::Emitter out;
  out << YAML::BeginMap;

	// When l!=0 we have 5 additional waypoints for the length
	data_saver::emit_value(out, "num_waypoints", (2+M)+N+1);
	// Generate waypoints for the line upwards 
	for(int i=0; i<M; ++i){
		waypoint_string = "waypoint_" + std::to_string(i+1);

		// x position = original_x + i*dl
		values[0] = hand_init[0] + r;
		values[1] = hand_init[1];
		values[2] = hand_init[2] + i*dl;
		values[3] = q_init.x(); values[4] = q_init.y(); values[5] = q_init.z(); values[6] = q_init.w();

		emit_7dof(out, waypoint_string, values);
	}
	// Generate waypoints for the arc
	for(int i=0; i<=N; ++i){
		waypoint_string = "waypoint_" + std::to_string(i+M+1);

		// x position = original_x + cos(i*dtheta)*radius
		values[0] = hand_init[0] + cos(static_cast<double>(i)*dtheta)*r;
		// y position = original_y + cos(i*dtheta)*radius
		values[1] = hand_init[1] + sin(static_cast<double>(i)*dtheta)*r;
		// z position = original_z
		values[2] = hand_init[2] + l;
		// Get the updated quaternion
		q.x() = 0; q.y() = 0; q.z() = sin((static_cast<double>(i)*dtheta) / 2); q.w() = cos((static_cast<double>(i)*dtheta) / 2);
		q_i = q * q_init;

		values[3] = q_i.x(); values[4] = q_i.y(); values[5] = q_i.z(); values[6] = q_i.w();

		emit_7dof(out, waypoint_string, values);
	}
  // Generate waypoints for the line downwards
  for(int i=0; i<M; ++i){
      waypoint_string = "waypoint_" + std::to_string(i+M+N+1);

      // x position = original_x + i*dl
      values[0] = hand_init[0] + cos(theta)*r;
      values[1] = hand_init[1] + sin(theta)*r;
      values[2] = hand_init[2] + l - i*dl;
      values[3] = q_i.x(); values[4] = q_i.y(); values[5] = q_i.z(); values[6] = q_i.w();

      emit_7dof(out, waypoint_string, values);
    }


  out << YAML::EndMap;
	std::ofstream file_output_stream("bag_lift_turn_putdown_trajectory.yaml");
  file_output_stream << out.c_str();
}
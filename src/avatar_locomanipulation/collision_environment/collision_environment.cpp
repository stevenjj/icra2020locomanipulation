#include <avatar_locomanipulation/collision_environment/collision_environment.h>



CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj){
	valkyrie = val;
	object = obj;
	std::cout << "Collision Environment Created" << std::endl;
}

CollisionEnvironment::~CollisionEnvironment(){
}


void CollisionEnvironment::build_directed_vectors(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
  // Update the kinematics
  valkyrie->updateFullKinematics(q);
  object->updateFullKinematics(obj_config);

  // Define the map for val_world_positions
  std::map<std::string, Eigen::Vector3d> world_positions = find_world_positions();
  // Define a map for near_points
  std::map<std::string, Eigen::Vector3d> near_points = find_near_points(q, obj_config);
  // Define the difference vector
  Eigen::Vector3d difference;

  for(std::map<std::string, Eigen::Vector3d>::iterator it=near_points.begin(); it != near_points.end(); ++it){
  	std::cout << it->first << " and rfoot: " <<  std::endl;
  	difference = world_positions.find("rfoot")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lfoot: " <<  std::endl;
  	difference = world_positions.find("lfoot")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and rankle: " <<  std::endl;
  	difference = world_positions.find("rankle")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lankle: " <<  std::endl;
  	difference = world_positions.find("lankle")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and rknee: " <<  std::endl;
  	difference = world_positions.find("rknee")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lknee: " <<  std::endl;
  	difference = world_positions.find("lknee")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and rshoulder: " <<  std::endl;
  	difference = world_positions.find("rshoulder")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lshoulder: " <<  std::endl;
  	difference = world_positions.find("lshoulder")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and relbow: " <<  std::endl;
  	difference = world_positions.find("relbow")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lelbow: " <<  std::endl;
  	difference = world_positions.find("lelbow")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and rwrist: " <<  std::endl;
  	difference = world_positions.find("rwrist")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lwrist: " <<  std::endl;
  	difference = world_positions.find("lwrist")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and rhand: " <<  std::endl;
  	difference = world_positions.find("rhand")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and lhand: " <<  std::endl;
  	difference = world_positions.find("lhand")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  	std::cout << it->first << " and pelvis: " <<  std::endl;
  	difference = world_positions.find("pelvis")->second - it->second;
  	std::cout << "Magnitude of distance = " << difference.norm() << std::endl;
  	std::cout << "Normalized Direction: " << std::endl;
  	std::cout << difference.normalized() << std::endl;

  }


}

std::map<std::string, Eigen::Vector3d> CollisionEnvironment::find_world_positions(){
	// Define the map to fill
	std::map<std::string, Eigen::Vector3d> positions;

	// First we define the pos/ori of the frames of interest
  Eigen::Vector3d cur_pos; 
  Eigen::Quaternion<double> cur_ori;

  // Then we get the world pose and fill the map 
  valkyrie->getFrameWorldPose("pelvis", cur_pos, cur_ori);
  positions["pelvis"] = cur_pos;
  valkyrie->getFrameWorldPose("rightCOP_Frame", cur_pos, cur_ori);
  positions["rfoot"] = cur_pos;
  valkyrie->getFrameWorldPose("leftCOP_Frame", cur_pos, cur_ori);
  positions["lfoot"] = cur_pos;
  valkyrie->getFrameWorldPose("rightAnklePitch", cur_pos, cur_ori);
  positions["rankle"] = cur_pos;
  valkyrie->getFrameWorldPose("leftAnklePitch", cur_pos, cur_ori);
  positions["lankle"] = cur_pos;
  valkyrie->getFrameWorldPose("rightKneePitch", cur_pos, cur_ori);
  positions["rknee"] = cur_pos;
  valkyrie->getFrameWorldPose("leftKneePitch", cur_pos, cur_ori);
  positions["lknee"] = cur_pos;
  valkyrie->getFrameWorldPose("rightShoulderRoll", cur_pos, cur_ori);
  positions["rshoulder"] = cur_pos;
  valkyrie->getFrameWorldPose("leftShoulderRoll", cur_pos, cur_ori);
  positions["lshoulder"] = cur_pos;
  valkyrie->getFrameWorldPose("rightElbowPitch", cur_pos, cur_ori);
  positions["relbow"] = cur_pos;
  valkyrie->getFrameWorldPose("leftElbowPitch", cur_pos, cur_ori);
  positions["lelbow"] = cur_pos;
  valkyrie->getFrameWorldPose("rightWristRoll", cur_pos, cur_ori);
  positions["rwrist"] = cur_pos;
  valkyrie->getFrameWorldPose("leftWristRoll", cur_pos, cur_ori);
  positions["lwrist"] = cur_pos;
  valkyrie->getFrameWorldPose("rightPalm", cur_pos, cur_ori);
  positions["rhand"] = cur_pos;
  valkyrie->getFrameWorldPose("leftPalm", cur_pos, cur_ori);
  positions["lhand"] = cur_pos;

  return positions;
}


std::map<std::string, Eigen::Vector3d> CollisionEnvironment::find_near_points(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
	// Define a new RobotModel which will be the appended model
	std::shared_ptr<RobotModel> appended(new RobotModel() );

	// Prepare the models for appending
  valkyrie->geomModel.addAllCollisionPairs();
	object->geomModel.addAllCollisionPairs();
	// Removes all collision pairs as specified in the srdf_filename
	pinocchio::srdf::removeCollisionPairs(valkyrie->model, valkyrie->geomModel, valkyrie->srdf_filename, false);

	// Append the object onto the robot, and fill appended RobotModel
	pinocchio::appendModel(valkyrie->model, object->model, valkyrie->geomModel, object->geomModel, valkyrie->model.frames.size()-1, pinocchio::SE3::Identity(), appended->model, appended->geomModel);

	// Define the appended configuration vector
	Eigen::VectorXd appended_config(appended->model.nq);
	appended_config << q, obj_config;

	// Create new data and geomData as required after appending models
	//appended->data(appended->model);
	pinocchio::GeometryData geomData(appended->geomModel);

	// Update the full kinematics 
	appended->updateFullKinematics(appended_config);

	std::string object_link_name;
	std::map<std::string, Eigen::Vector3d> near;

  for(int i=0; i < object->geomModel.geometryObjects.size(); ++i){
  	object_link_name = object->geomModel.getGeometryName(i);
  	for(int j=0; j<appended->geomModel.collisionPairs.size(); ++j){
			pinocchio::CollisionPair id2 = appended->geomModel.collisionPairs[j];
			if(appended->geomModel.getGeometryName(id2.first) == "pelvis" && appended->geomModel.getGeometryName(id2.second) == object_link_name){
				pinocchio::computeDistance(appended->geomModel, geomData, appended->geomModel.findCollisionPair(id2));
				appended->dresult = geomData.distanceResults[j];
				near[object_link_name] = appended->dresult.nearest_points[1];
			} // end if
		} // end inner for
  } // end outer for

  return near;
}
	
	

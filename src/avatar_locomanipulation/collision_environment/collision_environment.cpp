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
  // // Define a map for near_points
  std::map<std::string, Eigen::Vector3d> near_points; 
  std::map<std::string, Eigen::Vector3d>::iterator near_it;
  // Define a map for map to body names
  std::map<std::string, std::string> map_to_body_names = make_map_to_body_vector();
  std::map<std::string, std::string>::iterator it;
  

  // Define the difference vector
  Eigen::Vector3d difference;

  for(it=map_to_body_names.begin(); it!=map_to_body_names.end(); ++it){
    near_points = find_near_points(q, obj_config, it->second);
    for(near_it=near_points.begin(); near_it!=near_points.end(); ++near_it){
      difference = world_positions.find(it->first)->second - near_it->second;
      dvector.from = near_it->first; dvector.to = it->first;
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();
      directed_vectors.push_back(dvector);
    }
  }
}

void CollisionEnvironment::build_self_directed_vectors(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
  // Update the kinematics
  valkyrie->updateFullKinematics(q);
  object->updateFullKinematics(obj_config);

  // Define the map for val_world_positions
  std::map<std::string, Eigen::Vector3d> world_positions = find_world_positions_subset();

  build_directed_vector_to_rhand(world_positions);
  build_directed_vector_to_lhand(world_positions);
  build_directed_vector_to_elbows(world_positions); 
  build_directed_vector_to_knees(world_positions);
  build_directed_vector_to_head(world_positions);

}



std::map<std::string, Eigen::Vector3d> CollisionEnvironment::find_world_positions(){
	// Define the map to fill
	std::map<std::string, Eigen::Vector3d> positions;

	// First we define the pos/ori of the frames of interest
  Eigen::Vector3d cur_pos; 
  Eigen::Quaternion<double> cur_ori;

  std::map<std::string, std::string> map_to_frame_names = make_map_to_frame_vector();
  std::map<std::string, std::string>::iterator it;

  for(it=map_to_frame_names.begin(); it!=map_to_frame_names.end(); ++it){
    valkyrie->getFrameWorldPose(it->second, cur_pos, cur_ori);
    positions[it->first] = cur_pos;
  }

  return positions;
}

std::map<std::string, Eigen::Vector3d> CollisionEnvironment::find_world_positions_subset(){
  // Define the map to fill
  std::map<std::string, Eigen::Vector3d> positions_subset;

  // First we define the pos/ori of the frames of interest
  Eigen::Vector3d cur_pos; 
  Eigen::Quaternion<double> cur_ori;

  std::map<std::string, std::string> map_to_frame_names_subset = make_map_to_frame_vector_subset();
  std::map<std::string, std::string>::iterator it;

  for(it=map_to_frame_names_subset.begin(); it!=map_to_frame_names_subset.end(); ++it){
    valkyrie->getFrameWorldPose(it->second, cur_pos, cur_ori);
    positions_subset[it->first] = cur_pos;
  }

  return positions_subset;
}



std::map<std::string, Eigen::Vector3d> CollisionEnvironment::find_near_points(Eigen::VectorXd & q, Eigen::VectorXd & obj_config, std::string name){
	// Define map for returning
	std::map<std::string, Eigen::Vector3d> near;

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
	appended->common_initialization();
	pinocchio::GeometryData geomData(appended->geomModel);

	// Update the full kinematics 
	appended->updateFullKinematics(appended_config);

	std::string object_link_name;

  for(int i=0; i < object->geomModel.geometryObjects.size(); ++i){
  	object_link_name = object->geomModel.getGeometryName(i);
  	for(int j=0; j<appended->geomModel.collisionPairs.size(); ++j){
			pinocchio::CollisionPair id2 = appended->geomModel.collisionPairs[j];
			if((appended->geomModel.getGeometryName(id2.first) == name && appended->geomModel.getGeometryName(id2.second) == object_link_name)){
				pinocchio::computeDistance(appended->geomModel, geomData, appended->geomModel.findCollisionPair(id2));
				appended->dresult = geomData.distanceResults[j];
				near[object_link_name] = appended->dresult.nearest_points[1];
			} // end if
		} // end inner for
  } // end outer for

  return near;
}
	
	
void CollisionEnvironment::compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
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
	appended->common_initialization();
	pinocchio::GeometryData geomData(appended->geomModel);

	// Update the full kinematics 
	appended->updateFullKinematics(appended_config);

	int j, k;

	pinocchio::computeCollisions(appended->model, *appended->data, appended->geomModel, geomData, appended_config);

	for(j=0; j<geomData.collisionResults.size(); j++)
	{
		appended->result = geomData.collisionResults[j];
		pinocchio::CollisionPair id2 = appended->geomModel.collisionPairs[j];
		appended->result.getContacts(appended->contacts);
			if(appended->contacts.size() != 0)
			{
				for(k=0; k<appended->contacts.size(); k++)
      			{
      				std::cout << "Contact Found Between: " << appended->geomModel.getGeometryName(id2.first) << " and " << appended->geomModel.getGeometryName(id2.second) << std::endl;
        			std::cout << "position: " << appended->contacts[k].pos << std::endl;
        			std::cout << "-------------------" << std::endl;
      			}
			}
	}
}


void CollisionEnvironment::build_directed_vector_to_rhand(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d rhand = world_positions.find("rhand")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;
  from_names.push_back("lhand");
  from_names.push_back("lelbow");
  from_names.push_back("rknee");
  from_names.push_back("lknee");
  from_names.push_back("pelvis");
  from_names.push_back("torso");
  from_names.push_back("head");

  for(int i=0; i<from_names.size(); ++i){
    from = world_positions.find(from_names[i])->second;
    difference = rhand - from;
    dvector.from = from_names[i]; dvector.to = "rhand";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    directed_vectors.push_back(dvector);
  }
}


void CollisionEnvironment::build_directed_vector_to_lhand(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d lhand = world_positions.find("lhand")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;
  from_names.push_back("rhand");
  from_names.push_back("relbow");
  from_names.push_back("rknee");
  from_names.push_back("lknee");
  from_names.push_back("pelvis");
  from_names.push_back("torso");
  from_names.push_back("head");

  for(int i=0; i<from_names.size(); ++i){
    from = world_positions.find(from_names[i])->second;
    difference = lhand - from;
    dvector.from = from_names[i]; dvector.to = "lhand";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    directed_vectors.push_back(dvector);
  }
}


void CollisionEnvironment::build_directed_vector_to_elbows(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d lelbow = world_positions.find("lelbow")->second;
  Eigen::Vector3d relbow = world_positions.find("relbow")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;
  from_names.push_back("rknee");
  from_names.push_back("lknee");
  from_names.push_back("pelvis");
  from_names.push_back("torso");

  for(int i=0; i<from_names.size(); ++i){
    from = world_positions.find(from_names[i])->second;
    difference = relbow - from;
    dvector.from = from_names[i]; dvector.to = "relbow";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    directed_vectors.push_back(dvector);
  }

  for(int i=0; i<from_names.size(); ++i){
    from = world_positions.find(from_names[i])->second;
    difference = lelbow - from;
    dvector.from = from_names[i]; dvector.to = "lelbow";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    directed_vectors.push_back(dvector);
  }
}

void CollisionEnvironment::build_directed_vector_to_knees(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d lknee = world_positions.find("lknee")->second;
  Eigen::Vector3d rknee = world_positions.find("rknee")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;

  difference = lknee - rknee;
  dvector.from = "rknee"; dvector.to = "lknee";
  dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  directed_vectors.push_back(dvector);

  difference = rknee - lknee;
  dvector.from = "lknee"; dvector.to = "rknee";
  dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  directed_vectors.push_back(dvector);
}


void CollisionEnvironment::build_directed_vector_to_head(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d head = world_positions.find("head")->second;
  Eigen::Vector3d torso = world_positions.find("torso")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;

  difference = head - torso;
  dvector.from = "torso"; dvector.to = "head";
  dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  directed_vectors.push_back(dvector);
}


std::map<std::string, std::string> CollisionEnvironment::make_map_to_frame_vector(){
  std::map<std::string, std::string> map_to_frame_names;

  map_to_frame_names["rfoot"] = "rightCOP_Frame";
  map_to_frame_names["lfoot"] = "leftCOP_Frame";
  map_to_frame_names["rknee"] = "rightKneePitch";
  map_to_frame_names["lknee"] = "leftKneePitch";
  map_to_frame_names["pelvis"] = "pelvis";
  map_to_frame_names["torso"] = "torso";
  map_to_frame_names["rshoulder"] = "rightShoulderRoll";
  map_to_frame_names["lshoulder"] = "leftShoulderRoll";
  map_to_frame_names["relbow"] = "rightElbowPitch";
  map_to_frame_names["lelbow"] = "leftElbowPitch";
  map_to_frame_names["rhand"] = "rightPalm";
  map_to_frame_names["lhand"] = "leftPalm";
  map_to_frame_names["neck"] = "neckYaw";
  map_to_frame_names["head"] = "head";

  return map_to_frame_names;
}

std::map<std::string, std::string> CollisionEnvironment::make_map_to_frame_vector_subset(){
  std::map<std::string, std::string> map_to_frame_names_subset;

  map_to_frame_names_subset["rknee"] = "rightKneePitch";
  map_to_frame_names_subset["lknee"] = "leftKneePitch";
  map_to_frame_names_subset["pelvis"] = "pelvis";
  map_to_frame_names_subset["torso"] = "torso";
  map_to_frame_names_subset["relbow"] = "rightElbowPitch";
  map_to_frame_names_subset["lelbow"] = "leftElbowPitch";
  map_to_frame_names_subset["rhand"] = "rightPalm";
  map_to_frame_names_subset["lhand"] = "leftPalm";
  map_to_frame_names_subset["head"] = "head";

  return map_to_frame_names_subset;
}

std::map<std::string, std::string> CollisionEnvironment::make_map_to_body_vector(){
  
  std::map<std::string, std::string> map_to_body_names;

  map_to_body_names["rfoot"] = "rightFoot_0";
  map_to_body_names["lfoot"] = "leftFoot_0";
  map_to_body_names["rknee"] = "rightKneePitchLink_0";
  map_to_body_names["lknee"] = "leftKneePitchLink_0";
  map_to_body_names["pelvis"] = "pelvis_0";
  map_to_body_names["torso"] = "torso_0";
  map_to_body_names["rshoulder"] = "rightShoulderRollLink_0";
  map_to_body_names["lshoulder"] = "leftShoulderRollLink_0";
  map_to_body_names["relbow"] = "rightForearmLink_0";
  map_to_body_names["lelbow"] = "leftForearmLink_0";
  map_to_body_names["rhand"] = "rightPalm_0";
  map_to_body_names["lhand"] = "leftPalm_0";
  map_to_body_names["neck"] = "neckYawLink_0";
  map_to_body_names["head"] = "head_0";

  return map_to_body_names;
}
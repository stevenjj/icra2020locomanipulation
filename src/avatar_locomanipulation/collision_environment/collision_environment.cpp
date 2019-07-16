#include <avatar_locomanipulation/collision_environment/collision_environment.h>



CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj){
	valkyrie = val;
	object = obj;
	std::cout << "Collision Environment Created" << std::endl;
}

CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val){
  valkyrie = val;
  std::cout << "Collision Environment Created Only Val" << std::endl;
}



CollisionEnvironment::~CollisionEnvironment(){
}



std::shared_ptr<RobotModel> CollisionEnvironment::append_models(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
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

  // Update the full kinematics 
  appended->updateFullKinematics(appended_config);

  return appended;
}




std::map<std::string, Eigen::Vector3d> CollisionEnvironment::find_world_positions_subset(){
  // Define the map to fill
  std::map<std::string, Eigen::Vector3d> positions_subset;

  // First we define the pos/ori of the frames of interest
  Eigen::Vector3d cur_pos; 
  Eigen::Quaternion<double> cur_ori;

  std::map<std::string, std::string> map_to_frame_names_subset = make_map_to_frame_names_subset();
  std::map<std::string, std::string>::iterator it;

  for(it=map_to_frame_names_subset.begin(); it!=map_to_frame_names_subset.end(); ++it){
    valkyrie->getFrameWorldPose(it->second, cur_pos, cur_ori);
    positions_subset[it->first] = cur_pos;
  }

  return positions_subset;
}



void CollisionEnvironment::find_self_near_points(std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points, pinocchio::GeometryData & geomData){
  
  std::string to_link_name = list[0];
  std::string from_link_name;

  for(int i=1; i<list.size(); ++i){
    from_link_name = list[i];
    bool tmp = false;
    for(int j=0; j<valkyrie->geomModel.collisionPairs.size(); ++j){
      pinocchio::CollisionPair id2 = valkyrie->geomModel.collisionPairs[j];
      if((valkyrie->geomModel.getGeometryName(id2.first) == to_link_name && valkyrie->geomModel.getGeometryName(id2.second) == from_link_name)){
        valkyrie->dresult = pinocchio::computeDistance(valkyrie->geomModel, geomData, valkyrie->geomModel.findCollisionPair(id2));
        from_near_points[from_link_name] = valkyrie->dresult.nearest_points[2];
        to_near_points[from_link_name] = valkyrie->dresult.nearest_points[0];
        tmp = true;
        std::cout << "s1" << std::endl;
      } // First if closed
      else if((valkyrie->geomModel.getGeometryName(id2.first) == from_link_name && valkyrie->geomModel.getGeometryName(id2.second) == to_link_name)){
        valkyrie->dresult = pinocchio::computeDistance(valkyrie->geomModel, geomData, valkyrie->geomModel.findCollisionPair(id2));
        from_near_points[from_link_name] = valkyrie->dresult.nearest_points[0];
        to_near_points[from_link_name] = valkyrie->dresult.nearest_points[1];
        tmp = true;
        std::cout << "s2" << std::endl;
      } // else if closed
    } // Inner for closed
    if(tmp == false){
        std::cout << "Collision Pair: " << to_link_name << " and " << from_link_name << " not found" << std::endl;
      }
  } // Outer for closed

  // DEBUG
  std::map<std::string, Eigen::Vector3d>::iterator it = from_near_points.begin();
  std::map<std::string, Eigen::Vector3d>::iterator it2 = to_near_points.begin();
  for(it; it!=from_near_points.end(); ++it){
    std::cout << "Near point on " << it->first << " (from link) is \n" << it->second << std::endl;
    std::cout << "and corresponding near point on " << list[0] << " (to link) is \n" << it2->second << std::endl;
    std::cout << "------------------------" << std::endl;
    ++it2;
  }

}
	
	
void CollisionEnvironment::compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
  // Define the new appended model and fill it with the current configuration
  std::shared_ptr<RobotModel> appended = append_models(q, obj_config);

  // Build the appended_config for collision computation
  Eigen::VectorXd appended_config(appended->model.nq);
  appended_config << q, obj_config;

  // Define the geomData
	pinocchio::GeometryData geomData(appended->geomModel);

	int j, k;

  // Compute all collisions
	pinocchio::computeCollisions(appended->model, *appended->data, appended->geomModel, geomData, appended_config);

  // Loop thru results and print them
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




void CollisionEnvironment::build_directed_vector_to_rhand(pinocchio::GeometryData & geomData){
  
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  std::vector<std::string> collision_names;
  collision_names.push_back("rightPalm_0");
  collision_names.push_back("leftPalm_0");
  collision_names.push_back("leftElbowNearLink_0");// left elbow
  collision_names.push_back("rightKneeNearLink_0");// right knee
  collision_names.push_back("leftKneeNearLink_0");// left knee
  collision_names.push_back("head_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points, geomData);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // get the difference between near_points
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = it->first; dvector.to = "rightPalm_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }
  
  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}


void CollisionEnvironment::build_directed_vector_to_lhand(){
  
  // std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;
  // std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // std::vector<std::string> collision_names;
  // collision_names.push_back("leftPalm_0");
  // collision_names.push_back("rightPalm_0");
  // collision_names.push_back("righttElbowNearLink_0");// left elbow
  // collision_names.push_back("rightKneeNearLink_0");// right knee
  // collision_names.push_back("leftKneeNearLink_0");// left knee
  // collision_names.push_back("head_0");
  // collision_names.push_back("pelvis_0");
  // collision_names.push_back("torso_0");

  // Eigen::Vector3d difference;

  // find_self_near_points(collision_names, from_near_points, to_near_points);

  // it2 = to_near_points.begin();

  // for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
  //   difference = it2->second - it->second;
  //   // Fill the dvector and push back
  //   dvector.from = it->first; dvector.to = "leftPalm_0";
  //   dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  //   self_directed_vectors.push_back(dvector);
  //   ++it2;
  // }
  
  // std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
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
    self_directed_vectors.push_back(dvector);
  }

  for(int i=0; i<from_names.size(); ++i){
    from = world_positions.find(from_names[i])->second;
    difference = lelbow - from;
    dvector.from = from_names[i]; dvector.to = "lelbow";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
  }
}



void CollisionEnvironment::build_directed_vector_to_rknee(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d rknee = world_positions.find("rknee")->second;
  Eigen::Vector3d lknee = world_positions.find("lknee")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;

  difference = rknee - lknee;
  dvector.from = "lknee"; dvector.to = "rknee";
  dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  self_directed_vectors.push_back(dvector);
}

void CollisionEnvironment::build_directed_vector_to_lknee(std::map<std::string, Eigen::Vector3d> world_positions){
  Eigen::Vector3d lknee = world_positions.find("lknee")->second;
  Eigen::Vector3d rknee = world_positions.find("rknee")->second;

  Eigen::Vector3d from, difference, direction;
  double magnitude;
  std::vector<std::string> from_names;

  difference = lknee - rknee;
  dvector.from = "rknee"; dvector.to = "lknee";
  dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  self_directed_vectors.push_back(dvector);
}


void CollisionEnvironment::build_directed_vector_to_head(){
  
  // s:map<std::string, Eigen::Vector3d> from_near_points, to_near_points;
  // std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // std::vector<std::string> collision_names;
  // collision_names.push_back("head_0");
  // collision_names.push_back("torso_0");

  // Eigen::Vector3d difference;

  // find_self_near_points(collision_names, from_near_points, to_near_points);

  // it2 = to_near_points.begin();

  // for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
  //   difference = it2->second - it->second;
  //   // Fill the dvector and push back
  //   dvector.from = "torso_0"; dvector.to = "head_0";
  //   dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  //   self_directed_vectors.push_back(dvector);
  //   ++it2;
  // }

  // std::couttd::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;
  // std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // std::vector<std::string> collision_names;
  // collision_names.push_back("head_0");
  // collision_names.push_back("torso_0");

  // Eigen::Vector3d difference;

  // find_self_near_points(collision_names, from_near_points, to_near_points);

  // it2 = to_near_points.begin();

  // for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
  //   difference = it2->second - it->second;
  //   // Fill the dvector and push back
  //   dvector.from = "torso_0"; dvector.to = "head_0";
  //   dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
  //   self_directed_vectors.push_back(dvector);
  //   ++it2;
  // }

  // std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}





std::map<std::string, std::string> CollisionEnvironment::make_map_to_frame_names_subset(){
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



std::map<std::string, std::string> CollisionEnvironment::make_map_to_collision_body_names(){
  
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



std::vector<Eigen::Vector3d> CollisionEnvironment::self_collision_dx(){
  double Potential;
  std::vector<Eigen::Vector3d> dxs;
  Eigen::MatrixXd J_out(6, valkyrie->getDimQdot()); J_out.fill(0);;
  std::map<std::string, std::string> map_to_frame_names_subset = make_map_to_frame_names_subset();

  for(int k=0; k<self_directed_vectors.size(); ++k){
    Potential = (1/(self_directed_vectors[k].magnitude)) - (1/(safety_dist));

    if(Potential <= 0) Potential = 0;

    Eigen::Vector3d dx = (std::min(5.0 ,Potential))*(-self_directed_vectors[k].direction);

    dxs.push_back(dx);    
  }

  return dxs;

}


void CollisionEnvironment::set_safety_distance(double & safety_dist_in){
  safety_dist = safety_dist_in;
}
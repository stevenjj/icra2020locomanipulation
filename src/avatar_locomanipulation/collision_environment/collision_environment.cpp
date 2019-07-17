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



void CollisionEnvironment::find_self_near_points(std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points){
  
  // first name in the vector is the link to which we want to get near_point pairs
  std::string to_link_name = list[0];
  std::string from_link_name;

  for(int i=1; i<list.size(); ++i){
    // iterating thru rest of list, we get pairs with each of the other links
    from_link_name = list[i];

    // loop thru all of the collision pairs
    for(int j=0; j<valkyrie->geomModel.collisionPairs.size(); ++j){
      // grab this collision pair
      pinocchio::CollisionPair id2 = valkyrie->geomModel.collisionPairs[j];

      // check if pair.first and pair.second are our pair
      if((valkyrie->geomModel.getGeometryName(id2.first) == to_link_name && valkyrie->geomModel.getGeometryName(id2.second) == from_link_name)){
        
        // if this is our pair, computeDistance
        valkyrie->dresult = pinocchio::computeDistance(valkyrie->geomModel, *(valkyrie->geomData), valkyrie->geomModel.findCollisionPair(id2));
        
        // fill this map with nearest point on from object 
          // (i.e nearest point on link list[i] to link list[0])
        from_near_points[from_link_name] = valkyrie->dresult.nearest_points[1];
        
        // fill this map with nearest point on to object 
          // (i.e nearest point on link list[0] to link list[i])       
        to_near_points[from_link_name] = valkyrie->dresult.nearest_points[0];
      } // First if closed
      
      // check if pair.second and pair.first are our pair 
      //  (note that we do not know a priori which is pair.first and pair.second respectively)
      else if((valkyrie->geomModel.getGeometryName(id2.first) == from_link_name && valkyrie->geomModel.getGeometryName(id2.second) == to_link_name)){
        
        // if this is our pair, computeDistance
        valkyrie->dresult = pinocchio::computeDistance(valkyrie->geomModel, *(valkyrie->geomData), valkyrie->geomModel.findCollisionPair(id2));
        
        // fill this map with nearest point on from object 
          // (i.e nearest point on link list[i] to link list[0])
        from_near_points[from_link_name] = valkyrie->dresult.nearest_points[0];
        
        // fill this map with nearest point on to object 
          // (i.e nearest point on link list[0] to link list[i])
        to_near_points[from_link_name] = valkyrie->dresult.nearest_points[1];
      
      } // else if closed

    } // Inner for closed (i.e for this pair we have found the collisionpair and filled our map)

  } // Outer for closed (i.e we have found nearest_point pairs for every link in our input list)

}
	
	
void CollisionEnvironment::compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
  // Define the new appended model and fill it with the current configuration
  std::shared_ptr<RobotModel> appended = append_models(q, obj_config);

  // Build the appended_config for collision computation
  Eigen::VectorXd appended_config(appended->model.nq);
  appended_config << q, obj_config;

	int j, k;

  // Compute all collisions
	pinocchio::computeCollisions(appended->model, *appended->data, appended->geomModel, *appended->geomData, appended_config);

  // Loop thru results and print them
	for(j=0; j< appended->geomModel.collisionPairs.size(); j++)
	{
		appended->result = (*appended->geomData).collisionResults[j];
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




void CollisionEnvironment::build_directed_vector_to_rhand(){
  
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("rightPalm_0");
  collision_names.push_back("leftPalm_0");
  collision_names.push_back("leftElbowNearLink_0");
  collision_names.push_back("rightKneeNearLink_0");
  collision_names.push_back("leftKneeNearLink_0");
  collision_names.push_back("head_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  // fill our two maps
  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    std::cout << "to_near_points it2->second \n" << it2->second << std::endl;
    std::cout << "from_near_points it->second \n" << it->second << std::endl;

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
  
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;


  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("leftPalm_0");
  collision_names.push_back("rightPalm_0");
  collision_names.push_back("rightElbowNearLink_0");// left elbow
  collision_names.push_back("rightKneeNearLink_0");// right knee
  collision_names.push_back("leftKneeNearLink_0");// left knee
  collision_names.push_back("head_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // get difference between near_points
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = it->first; dvector.to = "leftPalm_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }
  
  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}



void CollisionEnvironment::build_directed_vector_to_head(){
  
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("head_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = "torso_0"; dvector.to = "head_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }
  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}



void CollisionEnvironment::build_directed_vector_to_rknee(){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("rightKneeNearLink_0");
  collision_names.push_back("leftKneeNearLink_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = "leftKneeNearLink_0"; dvector.to = "rightKneeNearLink_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }

  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}

void CollisionEnvironment::build_directed_vector_to_lknee(){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("leftKneeNearLink_0");
  collision_names.push_back("rightKneeNearLink_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = "rightKneeNearLink_0"; dvector.to = "leftKneeNearLink_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }
  
  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}




void CollisionEnvironment::build_directed_vector_to_relbow(){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("rightElbowNearLink_0");
  collision_names.push_back("rightKneeNearLink_0");
  collision_names.push_back("leftKneeNearLink_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = it->first; dvector.to = "rightElbowNearLink_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }

  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
}


void CollisionEnvironment::build_directed_vector_to_lelbow(){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;
  collision_names.push_back("leftElbowNearLink_0");
  collision_names.push_back("rightKneeNearLink_0");
  collision_names.push_back("leftKneeNearLink_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = it->first; dvector.to = "leftElbowNearLink_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    self_directed_vectors.push_back(dvector);
    ++it2;
  }

  std::cout << "self_directed_vectors.size(): " << self_directed_vectors.size() << std::endl;
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



std::vector<Eigen::Vector3d> CollisionEnvironment::self_collision_dx(){
  double Potential;
  std::vector<Eigen::Vector3d> dxs;
  Eigen::MatrixXd J_out(6, valkyrie->getDimQdot()); J_out.fill(0);;
  std::map<std::string, std::string> map_to_frame_names_subset = make_map_to_frame_names_subset();

  for(int k=0; k<self_directed_vectors.size(); ++k){
    Potential = (1/(self_directed_vectors[k].magnitude)) - (1/(safety_dist));

    if(Potential <= 0) Potential = 0;

    Eigen::Vector3d dx = (std::min(0.1 ,Potential))*(-self_directed_vectors[k].direction);

    dxs.push_back(dx);    
  }

  return dxs;

}


void CollisionEnvironment::set_safety_distance(double & safety_dist_in){
  safety_dist = safety_dist_in;
}
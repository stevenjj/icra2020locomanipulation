#include <avatar_locomanipulation/collision_environment/collision_environment.h>



CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj){
	valkyrie = val;
	object = obj;
	std::cout << "Collision Environment Created" << std::endl;
  object_flag = true;
  map_collision_names_to_frame_names();
}

CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val){
  valkyrie = val;
  std::cout << "Collision Environment Created Only Val" << std::endl;
  object_flag = false;
  map_collision_names_to_frame_names();
}



CollisionEnvironment::~CollisionEnvironment(){
}



std::shared_ptr<RobotModel> CollisionEnvironment::append_models(){
  // Define a new RobotModel which will be the appended model
  std::shared_ptr<RobotModel> appended(new RobotModel() );

  // Prepare the models for appending
  valkyrie->geomModel.addAllCollisionPairs();
  object->geomModel.addAllCollisionPairs();
  // Removes all collision pairs as specified in the srdf_filename
  pinocchio::srdf::removeCollisionPairs(valkyrie->model, valkyrie->geomModel, valkyrie->srdf_filename, false);

  // Append the object onto the robot, and fill appended RobotModel
  pinocchio::appendModel(valkyrie->model, object->model, valkyrie->geomModel, object->geomModel, valkyrie->model.frames.size()-1, pinocchio::SE3::Identity(), appended->model, appended->geomModel);

  appended->appended_initialization();

  // Define the appended configuration vector
  appended->q_current << valkyrie->q_current, object->q_current;

  // Update the full kinematics 
  appended->updateFullKinematics(appended->q_current);
  appended->updateGeometry(appended->q_current);

  return appended;
}




void CollisionEnvironment::find_self_near_points(std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points){
  
  // first name in the vector is the link to which we want to get near_point pairs
  std::string to_link_name = list[0];
  std::string from_link_name;

  from_near_points.clear();
  to_near_points.clear();

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



void CollisionEnvironment::find_object_near_points(std::shared_ptr<RobotModel> & appended, std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points){
  // first name in the vector is the link to which we want to get near_point pairs
  std::string to_link_name = list[0];
  std::string from_link_name;

  from_near_points.clear();
  to_near_points.clear();

  for(int i=1; i<list.size(); ++i){
    // iterating thru rest of list, we get pairs with each of the other links
    from_link_name = list[i];

    // loop thru all of the collision pairs
    for(int j=0; j<appended->geomModel.collisionPairs.size(); ++j){
      // grab this collision pair
      pinocchio::CollisionPair id2 = appended->geomModel.collisionPairs[j];

      // check if pair.first and pair.second are our pair
      if((appended->geomModel.getGeometryName(id2.first) == to_link_name && appended->geomModel.getGeometryName(id2.second) == from_link_name)){
        
        // if this is our pair, computeDistance
        appended->dresult = pinocchio::computeDistance(appended->geomModel, *(appended->geomData), appended->geomModel.findCollisionPair(id2));
        
        // fill this map with nearest point on from object 
          // (i.e nearest point on link list[i] to link list[0])
        from_near_points[from_link_name] = appended->dresult.nearest_points[1];
        
        // fill this map with nearest point on to object 
          // (i.e nearest point on link list[0] to link list[i])       
        to_near_points[from_link_name] = appended->dresult.nearest_points[0];
      } // First if closed
      
      // check if pair.second and pair.first are our pair 
      //  (note that we do not know a priori which is pair.first and pair.second respectively)
      else if((appended->geomModel.getGeometryName(id2.first) == from_link_name && appended->geomModel.getGeometryName(id2.second) == to_link_name)){
        
        // if this is our pair, computeDistance
        appended->dresult = pinocchio::computeDistance(appended->geomModel, *(appended->geomData), appended->geomModel.findCollisionPair(id2));
        
        // fill this map with nearest point on from object 
          // (i.e nearest point on link list[i] to link list[0])
        from_near_points[from_link_name] = appended->dresult.nearest_points[0];
        
        // fill this map with nearest point on to object 
          // (i.e nearest point on link list[0] to link list[i])
        to_near_points[from_link_name] = appended->dresult.nearest_points[1];
      
      } // else if closed

    } // Inner for closed (i.e for this pair we have found the collisionpair and filled our map)

  } // Outer for closed (i.e we have found nearest_point pairs for every link in our input list)


}
	
	
// void CollisionEnvironment::compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
//   // Define the new appended model and fill it with the current configuration
//   std::shared_ptr<RobotModel> appended = append_models();

//   // Build the appended_config for collision computation
//   Eigen::VectorXd appended_config(appended->model.nq);
//   appended_config << q, obj_config;

// 	int j, k;

//   // Compute all collisions
// 	pinocchio::computeCollisions(appended->model, *appended->data, appended->geomModel, *appended->geomData, appended_config);

//   // Loop thru results and print them
// 	for(j=0; j< appended->geomModel.collisionPairs.size(); j++)
// 	{
// 		appended->result = (*appended->geomData).collisionResults[j];
// 		pinocchio::CollisionPair id2 = appended->geomModel.collisionPairs[j];
// 		appended->result.getContacts(appended->contacts);
// 			if(appended->contacts.size() != 0)
// 			{
// 				for(k=0; k<appended->contacts.size(); k++)
//       			{
//       				std::cout << "Contact Found Between: " << appended->geomModel.getGeometryName(id2.first) << " and " << appended->geomModel.getGeometryName(id2.second) << std::endl;
//         			std::cout << "position: " << appended->contacts[k].pos << std::endl;
//         			std::cout << "-------------------" << std::endl;
//       			}
// 			}
// 	}
// }




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
  collision_names.push_back("leftShoulderRollLink_0");
  collision_names.push_back("leftForearmLink_0");
  collision_names.push_back("rightKneeNearLink_0");
  collision_names.push_back("leftKneeNearLink_0");
  collision_names.push_back("rightHipPitchLink_0");
  collision_names.push_back("leftHipPitchLink_0");
  collision_names.push_back("rightKneePitchLink_0");
  collision_names.push_back("leftKneePitchLink_0");
  collision_names.push_back("head_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  // fill our two maps
  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and rightPalm_0" << std::endl;
      get_dvector_collision_links(it->first, "rightPalm_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
      // get the difference between near_points
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = it->first; dvector.to = "rightPalm_0";
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
    }
  }

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
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
  collision_names.push_back("rightShoulderRollLink_0");
  collision_names.push_back("rightForearmLink_0");
  collision_names.push_back("rightKneeNearLink_0");// right knee
  collision_names.push_back("leftKneeNearLink_0");// left knee
  collision_names.push_back("rightHipPitchLink_0");
  collision_names.push_back("leftHipPitchLink_0");
  collision_names.push_back("rightKneePitchLink_0");
  collision_names.push_back("leftKneePitchLink_0");
  collision_names.push_back("head_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and leftPalm_0" << std::endl;
      get_dvector_collision_links(it->first, "leftPalm_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
      // get difference between near_points
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = it->first; dvector.to = "leftPalm_0";
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
    }
  }
  
  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
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
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and head_0" << std::endl;
      get_dvector_collision_links(it->first, "head_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = "torso_0"; dvector.to = "head_0";
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
    }
  }
  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
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
  collision_names.push_back("leftHipPitchLink_0");
  collision_names.push_back("leftKneePitchLink_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and rightKneeNearLink_0" << std::endl;
      get_dvector_collision_links(it->first, "rightKneeNearLink_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = "leftKneeNearLink_0"; dvector.to = "rightKneeNearLink_0";
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
    }
  }

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
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
  collision_names.push_back("rightHipPitchLink_0");
  collision_names.push_back("rightKneePitchLink_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and leftKneeNearLink_0" << std::endl;
      get_dvector_collision_links(it->first, "leftKneeNearLink_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = "rightKneeNearLink_0"; dvector.to = "leftKneeNearLink_0";
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
    }
  }
  
  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
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
  collision_names.push_back("leftForearmLink_0");
  collision_names.push_back("rightHipPitchLink_0");
  collision_names.push_back("leftHipPitchLink_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and rightElbowNearLink_0" << std::endl;
      get_dvector_collision_links(it->first, "rightElbowNearLink_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = it->first; dvector.to = "rightElbowNearLink_0";
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
    }
  }

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
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
  collision_names.push_back("rightForearmLink_0");
  collision_names.push_back("rightHipPitchLink_0");
  collision_names.push_back("leftHipPitchLink_0");
  collision_names.push_back("pelvis_0");
  collision_names.push_back("torso_0");

  Eigen::Vector3d difference;

  find_self_near_points(collision_names, from_near_points, to_near_points);

  it2 = to_near_points.begin();

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if(it->second == it2->second){
      std::cout << "Collision between " << it->first << " and leftElbowNearLink_0" << std::endl;
      get_dvector_collision_links(it->first, "leftElbowNearLink_0");
      ++it2;
    }

    // The typical case when two links are not in collision
    else{
    difference = it2->second - it->second;
    // Fill the dvector and push back
    dvector.from = it->first; dvector.to = "leftElbowNearLink_0";
    dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
    directed_vectors.push_back(dvector);
    ++it2;
    }
  }

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;
}



void CollisionEnvironment::build_object_directed_vectors(std::string & frame_name){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // fill list with collision_names[0] = name of link to which we want directed vectors
  // and collision_names[>0] = name of links from which we want directed vectors
  std::vector<std::string> collision_names;

  // used to build directed vectors
  Eigen::Vector3d difference;

  // gives a list of object link names
  std::vector<std::string> object_links = get_object_links();

  // Need to append the models for computeDistance to work inside find_object_near_points
  std::shared_ptr<RobotModel> appended = append_models();

  // we will build the directed vectors from each of the object links
  for(int i=0; i<object_links.size(); ++i){
    if(frame_name == "rightPalm"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("rightPalm_0");
    }

    if(frame_name == "leftPalm"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("leftPalm_0");
    } 

    if(frame_name == "head"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("head_0");
    }

    if(frame_name == "rightKneePitch"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("rightKneeNearLink_0");
    }

    if(frame_name == "leftKneePitch"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("leftKneeNearLink_0");
    }

    if(frame_name == "rightElbowPitch"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("rightElbowNearLink_0");
      collision_names.push_back("rightForearmLink_0");
    }

    if(frame_name == "leftElbowPitch"){
      collision_names.push_back(object_links[i]);
      collision_names.push_back("leftElbowNearLink_0");
      collision_names.push_back("leftForearmLink_0");
    }

    // Notice we reverse to and from near_points, because unlike in the self directed vectors,
    // we want vectors away from the collision_names[0]
    find_object_near_points(appended, collision_names, to_near_points, from_near_points);

    it2 = to_near_points.begin();

    for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
      // If nearest_point[1] = nearest_point[0], then the two links are in collision
      // and we need a different way to get a dvector
      if(it->second == it2->second){
        std::cout << "Collision between " << it->first << " and " << object_links[i] << std::endl;
        get_dvector_collision_links_appended(appended, object_links[i], it->first);
        ++it2;
      } // end if

      // The typical case when two links are not in collision
      else{
      difference = it2->second - it->second;
      // Fill the dvector and push back
      dvector.from = object_links[i]; dvector.to = it->first;
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();;
      directed_vectors.push_back(dvector);
      ++it2;
      } // end else

    } // end inner for

  } // end outer for

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;

}



std::vector<Eigen::Vector3d> CollisionEnvironment::get_collision_dx(){
  double Potential;
  std::vector<Eigen::Vector3d> dxs;
  Eigen::MatrixXd J_out(6, valkyrie->getDimQdot()); J_out.fill(0);

  for(int k=0; k<directed_vectors.size(); ++k){
    Potential = safety_dist*2 - (directed_vectors[k].magnitude);
    std::cout << "Potential before = " << Potential << std::endl;

    if(Potential <= safety_dist) Potential = 0;

    std::cout << "Potential after = " << Potential << std::endl;

    Eigen::Vector3d dx = (std::min(max_scaling_distance, Potential))*(directed_vectors[k].direction);

    dxs.push_back(dx);    
  }

  return dxs;

}


void CollisionEnvironment::set_safety_distance(double & safety_dist_in){
  safety_dist = safety_dist_in;
}

void CollisionEnvironment::set_max_scaling_distance(double & max_scaling_dist_in){
  max_scaling_distance = max_scaling_dist_in;
}



void CollisionEnvironment::get_dvector_collision_links(const std::string & from_name, const std::string & to_name){
  Eigen::Vector3d cur_pos_to, cur_pos_from, difference; 
  Eigen::Quaternion<double> cur_ori;
  
  valkyrie->getFrameWorldPose(collision_to_frame.find(to_name)->second, cur_pos_to, cur_ori);
  valkyrie->getFrameWorldPose(collision_to_frame.find(from_name)->second, cur_pos_from, cur_ori);

  difference = cur_pos_to - cur_pos_from;
  dvector.from = from_name; dvector.to = to_name;
  dvector.direction = difference.normalized(); dvector.magnitude = 0.005;
  directed_vectors.push_back(dvector);

}

void CollisionEnvironment::get_dvector_collision_links_appended(std::shared_ptr<RobotModel> & appended, const std::string & from_name, const std::string & to_name){
  std::cout << "get_dvector_collision_links_appended " << from_name << std::endl;

  Eigen::Vector3d cur_pos_to, cur_pos_from, difference; 
  Eigen::Quaternion<double> cur_ori;
  
  appended->getFrameWorldPose(collision_to_frame.find(to_name)->second, cur_pos_to, cur_ori);
  appended->getFrameWorldPose(collision_to_frame.find(from_name)->second, cur_pos_from, cur_ori);

  std::cout << "cur_pos_to : \n" << cur_pos_to << std::endl;
  std::cout << "cur_pos_from : \n" << cur_pos_from << std::endl;

  difference = cur_pos_to - cur_pos_from;
  dvector.from = from_name; dvector.to = to_name;
  dvector.direction = difference.normalized(); dvector.magnitude = 0.005;
  directed_vectors.push_back(dvector);

}

void CollisionEnvironment::map_collision_names_to_frame_names(){
  collision_to_frame["leftElbowNearLink_0"] = "leftElbowPitch";
  collision_to_frame["rightElbowNearLink_0"] = "rightElbowPitch";
  collision_to_frame["leftKneeNearLink_0"] = "leftKneePitch";
  collision_to_frame["rightKneeNearLink_0"] = "rightKneePitch";
  collision_to_frame["pelvis_0"] = "pelvis";
  collision_to_frame["torso_0"] = "torso";
  collision_to_frame["rightPalm_0"] = "rightPalm";
  collision_to_frame["leftPalm_0"] = "leftPalm_0";
  collision_to_frame["head_0"] = "head";

  std::string tmp;
  // if we added an object to the collision environment
  if(object_flag){
    // add the map from collision names to frame names
    for(int i=0; i<object->geomModel.geometryObjects.size(); ++i){
      tmp = object->geomModel.getGeometryName(i); tmp.pop_back(); tmp.pop_back();
      collision_to_frame[object->geomModel.getGeometryName(i)] = tmp;
    }
  }
  
}


std::vector<std::string> CollisionEnvironment::get_object_links(){
  // Will fill this vector with names of object collision body names
  std::vector<std::string> names;

  for(int i=0; i<object->geomModel.geometryObjects.size(); ++i){
    names.push_back(object->geomModel.getGeometryName(i));
  }

  return names;

}

#include <avatar_locomanipulation/collision_environment/collision_environment.h>



CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj){
	valkyrie = val;
	object = obj;
	std::cout << "Collision Environment Created" << std::endl;
  object_flag = true;
  eta = 1.0;
  map_collision_names_to_frame_names();
  generalize_build_self_directed_vectors();
  generalize_build_object_directed_vectors();
}

CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val){
  valkyrie = val;
  std::cout << "Collision Environment Created Only Val" << std::endl;
  object_flag = false;
  eta = 1.0;
  map_collision_names_to_frame_names();
  generalize_build_self_directed_vectors();
}



CollisionEnvironment::~CollisionEnvironment(){
}



std::shared_ptr<RobotModel> CollisionEnvironment::append_models(){
  // Define a new RobotModel which will be the appended model
  std::shared_ptr<RobotModel> appended(new RobotModel() );

  // Prepare the input models for appending
  valkyrie->geomModel.addAllCollisionPairs();
  object->geomModel.addAllCollisionPairs();
  // Removes all collision pairs as specified in the srdf_filename
  pinocchio::srdf::removeCollisionPairs(valkyrie->model, valkyrie->geomModel, valkyrie->srdf_filename, false);

  // Append the object onto the robot, and fill appended RobotModel
  pinocchio::appendModel(valkyrie->model, object->model, valkyrie->geomModel, object->geomModel, valkyrie->model.frames.size()-1, pinocchio::SE3::Identity(), appended->model, appended->geomModel);

  // Like common intialization but for appended objects
  // Difference is the initialization of geomData
  appended->appended_initialization();

  // Define the appended configuration vector
  appended->q_current << valkyrie->q_current, object->q_current;

  // Update the full kinematics 
  appended->enableUpdateGeomOnKinematicsUpdate(true);
  appended->updateFullKinematics(appended->q_current);

  return appended;
}




void CollisionEnvironment::find_self_near_points(std::string & to_link, const std::vector<std::string>  & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points){
  
  // first name in the vector is the link to which we want to get near_point pairs
  std::string to_link_name = to_link;
  std::string from_link_name;

  from_near_points.clear();
  to_near_points.clear();

  for(int i=0; i<list.size(); ++i){
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



void CollisionEnvironment::find_object_near_points(std::string & from_link, std::shared_ptr<RobotModel> & appended, std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points){
  // first name in the vector is the link to which we want to get near_point pairs
  std::string to_link_name = from_link;
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


void CollisionEnvironment::build_self_directed_vectors(const std::string & frame_name){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it;

  Eigen::Vector3d difference;

  std::string to_link = frame_name + "_0"; // Pinocchio Convention has _0 after collision links

  // fill our two maps
  find_self_near_points(to_link, link_to_collision_names.find(to_link)->second, from_near_points, to_near_points);

  for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
    
    // If nearest_point[1] = nearest_point[0], then the two links are in collision
    // and we need a different way to get a dvector
    if((it->second - to_near_points[it->first]).norm() <= 1e-6){
      std::cout << "Collision between " << it->first << " and " << to_link << std::endl;
      get_dvector_collision_links(it->first, to_link);
    }

    // The typical case when two links are not in collision
    else{
      // get the difference between near_points
      difference = to_near_points[it->first] - it->second;
      // Fill the dvector and push back
      dvector.from = collision_to_frame.find(it->first)->second; dvector.to = collision_to_frame.find(to_link)->second;
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();
      dvector.using_worldFramePose = false;
      directed_vectors.push_back(dvector);
    }
  }

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;

}
	



void CollisionEnvironment::build_object_directed_vectors(std::string & frame_name){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it, it2;

  // used to build directed vectors
  Eigen::Vector3d difference;

  // gives a list of object link names
  std::vector<std::string> object_links = get_object_links();

  // Need to append the models for computeDistance to work inside find_object_near_points
  std::shared_ptr<RobotModel> appended = append_models();

  // we will build the directed vectors from each of the object links
  for(int i=0; i<object_links.size(); ++i){
    

    // Notice we reverse to and from near_points, because unlike in the self directed vectors,
    // we want vectors away from the collision_names[0]
    find_object_near_points(object_links[i], appended, link_to_object_collision_names[frame_name], to_near_points, from_near_points);

    for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
      // If nearest_point[1] = nearest_point[0], then the two links are in collision
      // and we need a different way to get a dvector
      if( (it->second - to_near_points[it->first]).norm() <= 1e-6 ){
        std::cout << "Collision between " << it->first << " and " << object_links[i] << std::endl;
        get_dvector_collision_links_appended(appended, object_links[i], it->first);
        ++it2;
      } // end if

      // The typical case when two links are not in collision
      else{
      difference = to_near_points[it->first] - it->second;
      // Fill the dvector and push back
      dvector.from = collision_to_frame[object_links[i]]; dvector.to = collision_to_frame.find(it->first)->second;
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();
      dvector.using_worldFramePose = false;
      directed_vectors.push_back(dvector);
      ++it2;
      } // end else

    } // end inner for

  } // end outer for

  std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;

}


double CollisionEnvironment::get_collision_potential(){
  double Potential, temp;
  Potential = 0;
  closest = 0;
  std::vector<double> Potential_closestid;
  double safety_dist;

  temp = directed_vectors[0].magnitude;
  
  // Sort thru all of the directed vectors
  for(int j=1; j<directed_vectors.size(); ++j){
    // If in collision, this is the pair we want
    if(directed_vectors[j].using_worldFramePose){
      temp = directed_vectors[j].magnitude;
      closest = j;
      break;
    }
    // Else if the jth directed vector has magnitude less than temo
    else if(directed_vectors[j].magnitude < temp){
      // The jth directed vector is the closest pair 
      temp = directed_vectors[j].magnitude;
      closest = j;
    }
  }

  if(directed_vectors[closest].using_worldFramePose){
    safety_dist = safety_dist_collision;
    std::cout << "Collision between these links: (to, from) (" << directed_vectors[closest].to << ", " << directed_vectors[closest].from << ")" << std::endl;
  } else safety_dist = safety_dist_normal;


  if(directed_vectors[closest].magnitude < safety_dist){
    Potential = (1.0/2.0) * eta * std::pow(( (1/(directed_vectors[closest].magnitude)) - (1/(safety_dist)) ),2);
  }

  return Potential;

}



void CollisionEnvironment::set_safety_distance_normal(double safety_dist_normal_in){
  safety_dist_normal = safety_dist_normal_in;
}

void CollisionEnvironment::set_safety_distance_collision(double safety_dist_collision_in){
  safety_dist_collision = safety_dist_collision_in;
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
  dvector.from = collision_to_frame.find(from_name)->second; dvector.to = collision_to_frame.find(to_name)->second;
  dvector.direction = difference.normalized(); dvector.magnitude = 0.005;
  dvector.using_worldFramePose = true;
  directed_vectors.push_back(dvector);

}

void CollisionEnvironment::get_dvector_collision_links_appended(std::shared_ptr<RobotModel> & appended, const std::string & from_name, const std::string & to_name){

  Eigen::Vector3d cur_pos_to, cur_pos_from, difference; 
  Eigen::Quaternion<double> cur_ori;
  
  appended->getFrameWorldPose(collision_to_frame.find(to_name)->second, cur_pos_to, cur_ori);
  appended->getFrameWorldPose(collision_to_frame.find(from_name)->second, cur_pos_from, cur_ori);

  std::cout << "cur_pos_to : \n" << cur_pos_to << std::endl;
  std::cout << "cur_pos_from : \n" << cur_pos_from << std::endl;

  difference = cur_pos_to - cur_pos_from;
  dvector.from = collision_to_frame.find(from_name)->second; dvector.to = collision_to_frame.find(to_name)->second;
  dvector.direction = difference.normalized(); dvector.magnitude = 0.005;
  dvector.using_worldFramePose = true;
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
  collision_to_frame["leftPalm_0"] = "leftPalm";
  collision_to_frame["head_0"] = "head";
  collision_to_frame["rightHipUpperLink_0"] = "rightHipUpperLink";
  collision_to_frame["leftHipUpperLink_0"] = "leftHipUpperLink";
  collision_to_frame["rightForearmLink_0"] = "rightForearmLink";
  collision_to_frame["leftForearmLink_0"] = "leftForearmLink";
  collision_to_frame["rightShoulderRollLink_0"] = "rightShoulderRollLink";
  collision_to_frame["leftShoulderRollLink_0"] = "leftShoulderRollLink";
  collision_to_frame["rightHipPitchLink_0"] = "rightHipPitchLink";
  collision_to_frame["leftHipPitchLink_0"] = "leftHipPitchLink";
  collision_to_frame["rightKneePitchLink_0"] = "rightKneePitchLink";
  collision_to_frame["leftKneePitchLink_0"] = "leftKneePitchLink";

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

void CollisionEnvironment::generalize_build_self_directed_vectors(){
  link_to_collision_names["leftElbowNearLink_0"] = {"rightKneeNearLink_0", "leftKneeNearLink_0", "rightForearmLink_0","rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "pelvis_0", "torso_0"};
  
  link_to_collision_names["rightPalm_0"] = {"leftPalm_0", "leftElbowNearLink_0", "leftShoulderRollLink_0", "leftForearmLink_0", "rightKneeNearLink_0", "leftKneeNearLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "rightKneePitchLink_0", "leftKneePitchLink_0", "head_0", "pelvis_0", "torso_0"};
  
  link_to_collision_names["rightElbowNearLink_0"] = {"rightKneeNearLink_0", "leftKneeNearLink_0", "leftForearmLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "pelvis_0", "torso_0"};

  link_to_collision_names["leftKneeNearLink_0"] = {"rightKneeNearLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "rightKneePitchLink_0"};

  link_to_collision_names["rightKneeNearLink_0"] = {"leftKneeNearLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "leftKneePitchLink_0"};

  link_to_collision_names["head_0"] = {"torso_0"}; 

  link_to_collision_names["leftPalm_0"] = {"rightPalm_0", "rightElbowNearLink_0", "rightShoulderRollLink_0", "rightForearmLink_0", "rightKneeNearLink_0", "leftKneeNearLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "rightKneePitchLink_0", "leftKneePitchLink_0", "head_0", "pelvis_0", "torso_0"}; 
}


void CollisionEnvironment::generalize_build_object_directed_vectors(){
  link_to_object_collision_names["rightPalm"] = {"rightPalm_0"};

  link_to_object_collision_names["leftPalm"] = {"leftPalm_0"};

  link_to_object_collision_names["head"] = {"head_0"};

  link_to_object_collision_names["rightKneePitch"] = {"rightKneeNearLink_0"};

  link_to_object_collision_names["leftKneePitch"] = {"leftKneeNearLink_0"};

  link_to_object_collision_names["rightElbowPitch"] = {"rightElbowNearLink_0", "rightForearmLink_0"};

  link_to_object_collision_names["leftElbowPitch"] = {"leftElbowNearLink_0", "leftForearmLink_0"};

}

// std::vector<Eigen::Vector3d> CollisionEnvironment::get_collision_dx(){
//   double Potential;
//   std::vector<Eigen::Vector3d> dxs;
//   Eigen::MatrixXd J_out(6, valkyrie->getDimQdot()); J_out.fill(0);

//   for(int k=0; k<directed_vectors.size(); ++k){
//     if(directed_vectors[k].using_worldFramePose){
//       set_safety_distance(0.2);
//       std::cout << "Collision between these links: (to, from) (" << directed_vectors[k].to << ", " << directed_vectors[k].from << ")" << std::endl;
//     } 
//     else set_safety_distance(0.075);

//     Potential = safety_dist*2 - (directed_vectors[k].magnitude);
//     std::cout << "To link " << directed_vectors[k].to << " from link " << directed_vectors[k].from << std::endl;
//     std::cout << "Potential before = " << Potential << std::endl;

//     if(Potential <= safety_dist || directed_vectors[k].magnitude > safety_dist) Potential = 0;

//     std::cout << "Potential after = " << Potential << std::endl;

//     Eigen::Vector3d dx = (std::min(max_scaling_distance, Potential))*(directed_vectors[k].direction);

//     dxs.push_back(dx);    
//   }

//   return dxs;

// }




// void CollisionEnvironment::compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config){
//   // Define the new appended model and fill it with the current configuration
//   std::shared_ptr<RobotModel> appended = append_models();

//   // Build the appended_config for collision computation
//   Eigen::VectorXd appended_config(appended->model.nq);
//   appended_config << q, obj_config;

//  int j, k;

//   // Compute all collisions
//  pinocchio::computeCollisions(appended->model, *appended->data, appended->geomModel, *appended->geomData, appended_config);

//   // Loop thru results and print them
//  for(j=0; j< appended->geomModel.collisionPairs.size(); j++)
//  {
//    appended->result = (*appended->geomData).collisionResults[j];
//    pinocchio::CollisionPair id2 = appended->geomModel.collisionPairs[j];
//    appended->result.getContacts(appended->contacts);
//      if(appended->contacts.size() != 0)
//      {
//        for(k=0; k<appended->contacts.size(); k++)
//            {
//              std::cout << "Contact Found Between: " << appended->geomModel.getGeometryName(id2.first) << " and " << appended->geomModel.getGeometryName(id2.second) << std::endl;
//              std::cout << "position: " << appended->contacts[k].pos << std::endl;
//              std::cout << "-------------------" << std::endl;
//            }
//      }
//  }
// }
#include <avatar_locomanipulation/collision_environment/collision_environment.h>


CollisionEnvironment::CollisionEnvironment(std::shared_ptr<RobotModel> & val){
  valkyrie = val;

  // Prepare valkyrie for appending to appended
  valkyrie->geomModel.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(valkyrie->model, valkyrie->geomModel, valkyrie->srdf_filename, false);

  valkyrie->enableUpdateGeomOnKinematicsUpdate(true);
  valkyrie->updateFullKinematics(valkyrie->q_current);

  std::cout << "Collision Environment Created" << std::endl;
  // Indicates if there is an object
  object_flag = false;
  // Potential scaling factor
  eta = 1.0;
  // Set the object q counter
  object_q_counter = 0;
  // Indicates that appended will append a valkyrie model
  first_time = true;
  // create an empty RobotModel apended
  appended = std::shared_ptr<RobotModel> (new RobotModel() );
  // append empty appended with valkyrie
  append_models(); 
  // Builds a map from collision names (i.e. body_0) to frame names (i.e body)
  map_collision_names_to_frame_names();
  // Builds a map from collision body name (i.e. rightPalm_0) to list of valkyrie bodies from which we would want directed vectors
  generalize_build_self_directed_vectors();
  // Builds a map from collision body name (i.e. rightPalm_0) to list of object bodies from which we would want directed vectors
  generalize_build_object_directed_vectors();
}



CollisionEnvironment::~CollisionEnvironment(){
}



void CollisionEnvironment::append_models(){

  if(first_time){

    std::shared_ptr<RobotModel> app(new RobotModel() );

    // Append the object onto the robot, and fill appended RobotModel
    pinocchio::appendModel(appended->model, valkyrie->model, appended->geomModel, valkyrie->geomModel, appended->model.frames.size()-1, pinocchio::SE3::Identity(), app->model, app->geomModel);

    appended = app;

    // Like common intialization but for appended objects
    // Difference is the initialization of geomData
    appended->common_initialization(true);

    // Define the appended configuration vector
    appended->q_current = valkyrie->q_current;

    // Update the full kinematics 
    appended->enableUpdateGeomOnKinematicsUpdate(true);
    appended->updateFullKinematics(appended->q_current);

    first_time = false;

  }
  else{
    // temporarily holds the original appended->q_current
    Eigen::VectorXd temp;
    temp = appended->q_current;
    // create new temporary appended RobotModel for appendModel output
    std::shared_ptr<RobotModel> app(new RobotModel() );
    // prepare the object for appending
    object->geomModel.addAllCollisionPairs();
    // append the object into appended, such that its parent frame has the parent joint "universe"
    pinocchio::appendModel(appended->model, object->model, appended->geomModel, object->geomModel, 0, pinocchio::SE3::Identity(), app->model, app->geomModel);
    // now replace the appended with the appendModel output
    appended = app;
    // initialize this, since it does not yet have data, geomData
    //  a correctly sized q_current, etc.
    appended->common_initialization(true);
    // Fill the objects config inside of the appended model
    for(int i=0; i<object->q_current.size(); ++i){
      appended->q_current[i] = object->q_current[i];
    }
    // Fill the remainder of the appended with the original q_current
    int k=0;
    for(int j=object->q_current.size(); j<appended->q_current.size(); ++j){
      appended->q_current[j] = temp[k];
      ++k;
    }
    std::cout << "appended->getDimQ(): " << appended->getDimQ() << std::endl;
    std::cout << "appended->q_current.size(): " << appended->q_current.size() << std::endl;
    // Now that appended->q_current is filled update full kinematics
    appended->enableUpdateGeomOnKinematicsUpdate(true);
    appended->updateFullKinematics(appended->q_current);

    std::cout << "Object appended to appended [RobotModel] in [CollisionEnvironment]" << std::endl;
  }
    first_time = false;

}




void CollisionEnvironment::find_near_points(std::string & interest_link, const std::vector<std::string>  & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points){
  
  // first name in the vector is the link to which we want to get near_point pairs
  std::string to_link_name = interest_link;
  std::string from_link_name;

  from_near_points.clear();
  to_near_points.clear();

  for(int i=0; i<list.size(); ++i){
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





void CollisionEnvironment::build_self_directed_vectors(const std::string & frame_name, Eigen::VectorXd & q_update){
  // for clarity on these maps, see control flow in find_self_near_points function
  std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

  // initialize two iterators to be used in pushing to DirectedVectors struct
  std::map<std::string, Eigen::Vector3d>::iterator it;

  Eigen::Vector3d difference;

  std::string to_link = frame_name + "_0"; // Pinocchio Convention has _0 after collision links

  // Update the robot config for the given step of IK iteration
  for(int j=object_q_counter; j<appended->q_current.size(); ++j){
    appended->q_current[j] = q_update[j];
  }
  // Update full kinematics
  appended->enableUpdateGeomOnKinematicsUpdate(true);
  appended->updateFullKinematics(appended->q_current);

  // fill our two maps
  find_near_points(to_link, link_to_collision_names.find(to_link)->second, from_near_points, to_near_points);

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
  

void CollisionEnvironment::build_point_list_directed_vectors(const std::vector<Eigen::Vector3d> & point_list_in, Eigen::VectorXd & q_update){
  // used to build directed vectors
  Eigen::Vector3d difference;
  // Used to hold the current 3d pos of points in and worlFramePose of robotlinks
  Eigen::Vector3d cur_pos_to, cur_pos_from;
  // For getting results from getFrameWorldPose
  Eigen::Quaternion<double> cur_ori;
  // For sending to getFrameWorldPose
  std::string frame_name;
  // Get list of links of interest
  std::vector<std::string> names;
  names = make_point_collision_list();
  // For naming the from vectors
  char myString[2];

  directed_vectors.clear();

  // Update the robot config for the given step of IK iteration
  for(int j=object_q_counter; j<appended->q_current.size(); ++j){
    appended->q_current[j] = q_update[j];
  }
  // Update full kinematics
  appended->enableUpdateGeomOnKinematicsUpdate(true);
  appended->updateFullKinematics(appended->q_current);

  // we will build the directed vectors from each of the points in the list
  for(int i=0; i<point_list_in.size(); ++i){
    cur_pos_from = point_list_in[i];
    sprintf(myString, "%d", i);
    // loop thru all of the links of interest
    for(int j=0; j<names.size(); ++j){
      frame_name = names[j];
      // Get worldFramePose for this robot link
      appended->getFrameWorldPose(frame_name, cur_pos_to, cur_ori);
      difference = cur_pos_to - cur_pos_from;
      // Fill the dvector and push_back
      dvector.from = myString; dvector.to = frame_name;
      dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();
      dvector.using_worldFramePose = false;
      directed_vectors.push_back(dvector);
    }
    
  }
}

// void CollisionEnvironment::build_object_directed_vectors(std::string & frame_name, Eigen::VectorXd & q_update){
//   // for clarity on these maps, see control flow in find_self_near_points function
//   std::map<std::string, Eigen::Vector3d> from_near_points, to_near_points;

//   // initialize two iterators to be used in pushing to DirectedVectors struct
//   std::map<std::string, Eigen::Vector3d>::iterator it;

  // // used to build directed vectors
  // Eigen::Vector3d difference;

  // // Update the robot config for the given step of IK iteration
  // for(int j=object_q_counter; j<appended->q_current.size(); ++j){
  //   appended->q_current[j] = q_update[j];
  // }
  // // Update full kinematics
  // appended->enableUpdateGeomOnKinematicsUpdate(true);
  // appended->updateFullKinematics(appended->q_current);

//   // gives a list of object link names
//   std::vector<std::string> object_links = get_object_links();

  // // we will build the directed vectors from each of the object links
  // for(int i=0; i<object_links.size(); ++i){
    
  //   // Notice we reverse to and from near_points, because unlike in the self directed vectors,
  //   // we want vectors away from the collision_names[0]
  //   find_near_points(object_links[i], link_to_object_collision_names[frame_name], to_near_points, from_near_points);



  //   for(it=from_near_points.begin(); it!=from_near_points.end(); ++it){
  //     // If nearest_point[1] = nearest_point[0], then the two links are in collision
  //     // and we need a different way to get a dvector
  //     if( (it->second - to_near_points[it->first]).norm() <= 1e-6 ){
  //       std::cout << "Collision between " << it->first << " and " << object_links[i] << std::endl;
  //       get_dvector_collision_links(object_links[i], it->first);
  //     } // end if

  //     // The typical case when two links are not in collision
  //     else{
  //     difference = to_near_points[it->first] - it->second;
  //     // Fill the dvector and push back
  //     dvector.from = collision_to_frame[object_links[i]]; dvector.to = collision_to_frame.find(it->first)->second;
  //     dvector.direction = difference.normalized(); dvector.magnitude = difference.norm();
  //     dvector.using_worldFramePose = false;
  //     directed_vectors.push_back(dvector);
  //     } // end else

  //   } // end inner for

  // } // end outer for

//   std::cout << "directed_vectors.size(): " << directed_vectors.size() << std::endl;

//   // std::cout << "appended->model:\n" << appended->model << std::endl;
//   // std::cout << "appended->geomModel:\n" << appended->geomModel << std::endl;
//   // for (int k=0 ; k<appended->model.frames.size() ; ++k){
//   //   std::cout << "frame:" << k << " " << appended->model.frames[k].name << " : " << appended->data->oMf[k].translation().transpose() << std::endl;
//   // }

//   Eigen::Vector3d cur_pos_to, cur_pos_from; 
//   Eigen::Quaternion<double> cur_ori;
  
//   appended->getFrameWorldPose(collision_to_frame.find("base_link_0")->second, cur_pos_to, cur_ori);
//   std::cout << "current pose base_link_0:\n" << cur_pos_to << std::endl;
//   appended->getFrameWorldPose("rightThumbPitch3Link", cur_pos_to, cur_ori);
//   std::cout << "current pose rightThumbPitch3Link:\n" << cur_pos_to << std::endl;
//   appended->getFrameWorldPose(collision_to_frame.find("rightPalm_0")->second, cur_pos_to, cur_ori);
//   std::cout << "current pose rightPalm_0:\n" << cur_pos_to << std::endl;

//   for(int i=0; i<directed_vectors.size(); ++i){
//     std::cout << "directed_vectors[i].from: " << directed_vectors[i].from << std::endl;
//     std::cout << "directed_vectors[i].to: " << directed_vectors[i].to << std::endl;
//     std::cout << "directed_vectors[i].magnitude: " << directed_vectors[i].magnitude << std::endl;
//     std::cout << "directed_vectors[i].direction: \n" << directed_vectors[i].direction << std::endl;
//   }

  

// }


double CollisionEnvironment::get_collision_potential(){
  double Potential, temp;
  Potential = 0;
  closest = 0;
  double safety_dist;

  temp = directed_vectors[0].magnitude;
  
  // We want to find the index of the directed vector with lowest magnitude
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

  // Set the local safety distance accordingly and output if there is a collision
  if(directed_vectors[closest].using_worldFramePose){
    safety_dist = safety_dist_collision;
    std::cout << "Collision between these links: (to, from) (" << directed_vectors[closest].to << ", " << directed_vectors[closest].from << ")" << std::endl;
  } else safety_dist = safety_dist_normal;

  // Get the potential using the proper safety_distance
  if(directed_vectors[closest].magnitude < safety_dist){
    Potential = (1.0/2.0) * eta * std::pow(( (1/(directed_vectors[closest].magnitude)) - (1/(safety_dist)) ),2);
  }

  return Potential;

}



// void CollisionEnvironment::add_new_object(std::shared_ptr<RobotModel> & obj, const Eigen::VectorXd & q_start){
//   // Initialize the RobotModel
//   object = obj;
//   // Set the q_current
//   object->q_current = q_start;
//   // Keep track of nq
//   object_q_counter += q_start.size();
//   // Update kinematics and geom placements
//   object->enableUpdateGeomOnKinematicsUpdate(true);
//   object->updateFullKinematics(object->q_current);

//   // Tells append_models to add this to end of appended
//   first_time = false;

//   // Appends this to end of appended
//   append_models();

//   // Tells the collision to frame names to look through the object
//   object_flag = true;
//   // Adds this objects collision and frame names to collision_to_frame
//   map_collision_names_to_frame_names();

//   std::cout << "[RobotModel] Environmental Object Created and appended in [CollisionEnvironment]" << std::endl;
// }






void CollisionEnvironment::set_safety_distance_normal(double safety_dist_normal_in){
  safety_dist_normal = safety_dist_normal_in;
}

void CollisionEnvironment::set_safety_distance_collision(double safety_dist_collision_in){
  safety_dist_collision = safety_dist_collision_in;
}



void CollisionEnvironment::get_dvector_collision_links(const std::string & from_name, const std::string & to_name){
  Eigen::Vector3d cur_pos_to, cur_pos_from, difference; 
  Eigen::Quaternion<double> cur_ori;
  
  appended->getFrameWorldPose(collision_to_frame.find(to_name)->second, cur_pos_to, cur_ori);
  appended->getFrameWorldPose(collision_to_frame.find(from_name)->second, cur_pos_from, cur_ori);

  difference = cur_pos_to - cur_pos_from;
  dvector.from = collision_to_frame.find(from_name)->second; dvector.to = collision_to_frame.find(to_name)->second;
  dvector.direction = difference.normalized(); dvector.magnitude = 0.005;
  dvector.using_worldFramePose = true;
  directed_vectors.push_back(dvector);

}

std::vector<std::string> CollisionEnvironment::make_point_collision_list(){

  std::vector<std::string> names;
  names.push_back("torso");
  names.push_back("pelvis");
  names.push_back("head");
  names.push_back("rightPalm");
  names.push_back("leftPalm");
  names.push_back("rightKneePitch");
  names.push_back("leftKneePitch");
  names.push_back("rightElbowPitch");
  names.push_back("leftElbowPitch");
  names.push_back("rightForearmLink");
  names.push_back("leftForearmLink");
  names.push_back("rightHipUpperLink");
  names.push_back("leftHipUpperLink");

  return names;
}


void CollisionEnvironment::map_collision_names_to_frame_names(){
  collision_to_frame["leftElbowNearLink_0"] = "leftElbowPitch";
  collision_to_frame["rightElbowNearLink_0"] = "rightElbowPitch";
  collision_to_frame["leftKneeNearLink_0"] = "leftKneePitch";
  collision_to_frame["rightKneeNearLink_0"] = "rightKneePitch";
  collision_to_frame["pelvis_0"] = "pelvis";
  collision_to_frame["torso_0"] = "torso";
  collision_to_frame["torso_1"] = "torso";
  collision_to_frame["torso_2"] = "torso";
  collision_to_frame["torso_3"] = "torso";
  collision_to_frame["torso_4"] = "torso";
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

  for(int i=valkyrie->geomModel.geometryObjects.size(); i<appended->geomModel.geometryObjects.size(); ++i){
    names.push_back(appended->geomModel.getGeometryName(i));
  }

  return names;

}

void CollisionEnvironment::generalize_build_self_directed_vectors(){
  link_to_collision_names["leftElbowNearLink_0"] = {"rightKneeNearLink_0", "leftKneeNearLink_0", "rightForearmLink_0","rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "pelvis_0", "torso_0", "torso_1", "torso_2", "torso_3", "torso_4"};
  
  link_to_collision_names["rightPalm_0"] = {"leftPalm_0", "leftElbowNearLink_0", "leftShoulderRollLink_0", "leftForearmLink_0", "rightKneeNearLink_0", "leftKneeNearLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "rightKneePitchLink_0", "leftKneePitchLink_0", "head_0", "pelvis_0", "torso_0", "torso_1", "torso_2", "torso_3", "torso_4"};
  
  link_to_collision_names["rightElbowNearLink_0"] = {"rightKneeNearLink_0", "leftKneeNearLink_0", "leftForearmLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "pelvis_0", "torso_0", "torso_1", "torso_2", "torso_3", "torso_4"};

  link_to_collision_names["leftKneeNearLink_0"] = {"rightKneeNearLink_0", "rightHipPitchLink_0", "rightHipUpperLink_0", "rightKneePitchLink_0"};

  link_to_collision_names["rightKneeNearLink_0"] = {"leftKneeNearLink_0", "leftHipPitchLink_0", "leftHipUpperLink_0", "leftKneePitchLink_0"};

  link_to_collision_names["head_0"] = {"torso_0", "torso_1", "torso_2", "torso_3", "torso_4"}; 

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
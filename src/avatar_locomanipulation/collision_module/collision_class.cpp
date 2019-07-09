// Headers
#include <avatar_locomanipulation/collision_module/collision_class.h>


Collision::Collision()
{
	
}

Collision::~Collision()
{

}

void Collision::build_valkyrie_model_and_geom()
{
	filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf";
	srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";

	std::vector < std::string > packageDirs;
	std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

	packageDirs.push_back(meshDir);
	// Build URDF model
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
	// Build Geometry model
	pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs );
}

void Collision::build_box_planar_joint_model_and_geom()
{
	pinocchio::fcl::Transform3f tf1 (makeQuat (0, 0, 1, 0), pinocchio::fcl::Vec3f (0, 0, 0));
	pinocchio::SE3 placement = pinocchio::toPinocchioSE3(tf1);

	idx = model.addJoint(model.getJointId("universe"),pinocchio::JointModelPlanar(),pinocchio::SE3::Identity(),"planar1_joint");
	model.addJointFrame(idx);

	model.appendBodyToJoint(idx,pinocchio::Inertia::Random(),pinocchio::SE3::Identity());
	model.addBodyFrame("planar1_body", idx, pinocchio::SE3::Identity());

	boost::shared_ptr<pinocchio::fcl::Box> sample(new pinocchio::fcl::Box(1, 1, 1));
	pinocchio::Model::FrameIndex body_id_1 = model.getBodyId("planar1_body");
	pinocchio::Model::JointIndex joint_parent_1 = model.frames[body_id_1].parent;

	pinocchio::Model::JointIndex idx_geom1 = geomModel.addGeometryObject(pinocchio::GeometryObject("ff1_collision_object",
                                                                           model.getBodyId("planar1_body"),joint_parent_1,
                                                                           sample,placement, "", Eigen::Vector3d::Ones())
                                                            );
}

void Collision::build_cart_model_and_geom()
{
	filename = THIS_PACKAGE_PATH"models/test_cart.urdf";

	std::vector < std::string > packageDirs;
	std::string meshDir  = THIS_PACKAGE_PATH"models/cart/";

	packageDirs.push_back(meshDir);
	// Build URDF model
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
	// Build Geometry model
	pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs );
}

void Collision::append_models(std::shared_ptr<Collision> & parent, std::shared_ptr<Collision> & child, std::shared_ptr<Collision> & appended)
{
	parent->geomModel.addAllCollisionPairs();
	// Removes all collision pairs as specified in the srdf_filename
	pinocchio::srdf::removeCollisionPairs(parent->model, parent->geomModel, srdf_filename, false);

	pinocchio::appendModel(parent->model, child->model, parent->geomModel, child->geomModel, parent->model.frames.size()-1, pinocchio::SE3::Identity(), appended->model, appended->geomModel);
}

void Collision::set_configuration_vector(Eigen::VectorXd & config_vec)
{
	q.resize(model.nq);
	int i;

	for(i=0; i<model.nq; i++)
	{
		q[i] = config_vec[i];
	}

	pinocchio::Data data(model);
	pinocchio::GeometryData geomData(geomModel);
	pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
}


void Collision::compute_collision()
{
	int j, k;

	pinocchio::Data data(model);
	pinocchio::GeometryData geomData(geomModel);

	pinocchio::computeCollisions(model,data,geomModel,geomData,q);

	std::cout << "geomData.collisionResults.size(): " << geomData.collisionResults.size() << std::endl;

	for(j=0; j<geomData.collisionResults.size(); j++)
	{
		result = geomData.collisionResults[j];
		pinocchio::CollisionPair id2 = geomModel.collisionPairs[j];
		result.getContacts(contacts);
			if(contacts.size() != 0)
			{
				for(k=0; k<contacts.size(); k++)
      			{
      				std::cout << "Contact Found Between: " << geomModel.getGeometryName(id2.first) << " and " << geomModel.getGeometryName(id2.second) << std::endl;
        			std::cout << "position: " << contacts[k].pos << std::endl;
        			std::cout << "-------------------" << std::endl;
      			}
			}
	}
		
}


void Collision::compute_distances()
{
	int j;

	pinocchio::Data data(model);
	pinocchio::GeometryData geomData(geomModel);

	for(j=0; j<geomModel.collisionPairs.size(); j++)
	{
		pinocchio::CollisionPair id2 = geomModel.collisionPairs[j];
		pinocchio::computeDistance(geomModel, geomData, geomModel.findCollisionPair(id2));
		dresult = geomData.distanceResults[j];
		std::cout << "Minimum Distance Between: " << geomModel.getGeometryName(id2.first) << " and " << geomModel.getGeometryName(id2.second) << " = " << dresult.min_distance << std::endl;
	}
}

pinocchio::fcl::Quaternion3f Collision::makeQuat(double w, double x, double y, double z)
{
  pinocchio::fcl::Quaternion3f p;
  p.w() = w;
  p.x() = x;
  p.y() = y;
  p.z() = z;
  return p;
}

int Collision::get_nq()
{
	int i = model.nq;

	return i;
}


// Instead of this we can create a map of the joint name : world position
// going thru that map we need to create vectors from half to other half
// additionally, we need to create vectors from object to joints


void Collision::compute_near_point(std::string name1, std::string name2, Eigen::Vector3d & near_point, Eigen::VectorXd q_start)
{
	pinocchio::Data data(model);
	pinocchio::GeometryData geomData(geomModel);
	pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q_start);
	int j;

	for(j=0; j<geomModel.collisionPairs.size(); j++)
	{
		pinocchio::CollisionPair id2 = geomModel.collisionPairs[j];
		if (geomModel.getGeometryName(id2.first) == name1 && geomModel.getGeometryName(id2.second) == name2)
		{
			pinocchio::computeDistance(geomModel, geomData, geomModel.findCollisionPair(id2));
			dresult = geomData.distanceResults[j];
			near_point = dresult.nearest_points[1];
		}
		
	}
}


void Collision::get_position_of_joints(Eigen::VectorXd q_start, std::map<std::string, Eigen::Vector3d> & positions)
{

  // Initialize Robot Model
  ValkyrieModel valkyrie;

  // First we define the pos/ori of the frames of interest
  Eigen::Quaternion<double> rfoot_cur_ori;
  Eigen::Quaternion<double> lfoot_cur_ori;
  Eigen::Quaternion<double> rankle_cur_ori;
  Eigen::Quaternion<double> lankle_cur_ori;
  Eigen::Quaternion<double> rknee_cur_ori;
  Eigen::Quaternion<double> lknee_cur_ori;
  Eigen::Quaternion<double> pelvis_cur_ori;
  Eigen::Quaternion<double> rshoulder_cur_ori;
  Eigen::Quaternion<double> lshoulder_cur_ori;
  Eigen::Quaternion<double> relbow_cur_ori;
  Eigen::Quaternion<double> lelbow_cur_ori;
  Eigen::Quaternion<double> rwrist_cur_ori;
  Eigen::Quaternion<double> lwrist_cur_ori;
  Eigen::Quaternion<double> rhand_cur_ori;
  Eigen::Quaternion<double> lhand_cur_ori;

  valkyrie.getFrameWorldPose("pelvis", positions.find("pelvis")->second, pelvis_cur_ori);
  valkyrie.getFrameWorldPose("rightCOP_Frame", positions.find("rfoot")->second, rfoot_cur_ori);
  valkyrie.getFrameWorldPose("rightCOP_Frame", positions.find("lfoot")->second, lfoot_cur_ori);
  valkyrie.getFrameWorldPose("leftAnklePitch", positions.find("rankle")->second, rankle_cur_ori);
  valkyrie.getFrameWorldPose("rightAnklePitch", positions.find("lankle")->second, lankle_cur_ori);
  valkyrie.getFrameWorldPose("leftKneePitch", positions.find("rknee")->second, rknee_cur_ori);
  valkyrie.getFrameWorldPose("rightKneePitch", positions.find("lknee")->second, lknee_cur_ori);
  valkyrie.getFrameWorldPose("rightShoulderRoll", positions.find("rshoulder")->second, rshoulder_cur_ori);
  valkyrie.getFrameWorldPose("leftShoulderRoll", positions.find("lshoulder")->second, lshoulder_cur_ori);
  valkyrie.getFrameWorldPose("rightElbowPitch", positions.find("relbow")->second, relbow_cur_ori);
  valkyrie.getFrameWorldPose("leftElbowPitch", positions.find("lelbow")->second, lelbow_cur_ori);
  valkyrie.getFrameWorldPose("rightWristRoll", positions.find("rwrist")->second, rwrist_cur_ori);
  valkyrie.getFrameWorldPose("leftWristRoll", positions.find("lwrist")->second, lwrist_cur_ori);
  valkyrie.getFrameWorldPose("rightPalm", positions.find("rhand")->second, rhand_cur_ori);
  valkyrie.getFrameWorldPose("leftPalm", positions.find("lhand")->second, lhand_cur_ori);
}
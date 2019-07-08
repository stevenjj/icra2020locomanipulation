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


bool Collision::compute_collision()
{
	int j;

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
				return true;
			}
	}
	return false;	
}


void Collision::compute_distance()
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
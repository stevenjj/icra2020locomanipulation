#include <avatar_locomanipulation/collision_module/collision_class.h>
#include <iostream>

int main(int argc, char **argv){
	std::cout << "Collision Module Test" << std::endl;
	std::shared_ptr<Collision> val_object(new Collision() );
	std::shared_ptr<Collision> box_object(new Collision() );
	std::shared_ptr<Collision> appended_object(new Collision() );
	std::shared_ptr<Collision> cart_object(new Collision() );

	val_object->build_valkyrie_model_and_geom();
	box_object->build_box_planar_joint_model_and_geom();
	cart_object->build_cart_model_and_geom();

	Eigen::VectorXd val_config(val_object->get_nq());
	Eigen::VectorXd box_config(box_object->get_nq());
	Eigen::VectorXd cart_config(cart_object->get_nq());

	// floating base joints: x, y, z
	val_config[0] = 0.0;  val_config[1] = 0.0;  val_config[2] = 0.0;

	// floating base quaternion: qx, qy, qz, qw
	double theta = 0;//M_PI/4.0;	
	Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left	
	Eigen::Quaternion<double> quat_init; quat_init =  aa;
	val_config[3] = quat_init.x();// 0.0;	
	val_config[4] = quat_init.y(); //0.0;
	val_config[5] = quat_init.z(); //sin(theta/2.0);
	val_config[6] = quat_init.w(); //cos(theta/2.0);

	box_config << 0, 0, 1, 0;

	val_object->set_configuration_vector(val_config);
	box_object->set_configuration_vector(box_config);


	val_object->append_models(val_object, box_object, appended_object);

	Eigen::VectorXd appended_config(appended_object->get_nq());
	appended_config << val_config, box_config;
	appended_object->set_configuration_vector(appended_config);


	appended_object->compute_collision();
	// if(collide == true) std::cout << "collision" << std::endl;

}
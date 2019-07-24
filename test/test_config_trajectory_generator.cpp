#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>

// Standard
#include <iostream>
#include <math.h>
#include <cassert>

void test_config_trajectory_generator(){
  std::cout << "[Running Config Trajectory Generator Test]" << std::endl;

  std::cout << "[ConfigTrajectoryGenerator] Constructed" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  int N_resolution = 50;
  ConfigTrajectoryGenerator ctg(valkyrie_model, N_resolution);
}

int main(int argc, char ** argv){   
  test_config_trajectory_generator();
  return 0;
}
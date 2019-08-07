#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>

int main(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));

  FeasibilityDataGenerator feas_data_gen;
  feas_data_gen.setRobotModel(robot_model);

  return 0;
}
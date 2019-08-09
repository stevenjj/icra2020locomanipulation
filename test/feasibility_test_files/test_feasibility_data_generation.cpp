#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>

int main(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));

  FeasibilityDataGenerator feas_data_gen;
  feas_data_gen.setRobotModel(robot_model);

  unsigned int seed_number = 1;
  feas_data_gen.initializeSeed(seed_number);

  for(size_t i = 0; i < 10; i++){
    std::cout << static_cast<double>(rand()) / static_cast<double>(RAND_MAX)  << std::endl;
  }

  feas_data_gen.initializeStartingIKTasks();

  return 0;
}
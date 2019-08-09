#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>

int main(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));

  FeasibilityDataGenerator feas_data_gen;
  feas_data_gen.setRobotModel(robot_model);

  unsigned int seed_number = 1;
  feas_data_gen.initializeSeed(seed_number);

  for(size_t i = 0; i < 20; i++){
    std::cout << feas_data_gen.generateRandMinMax(0.0, 0.5) << std::endl;
  }

  feas_data_gen.initializeStartingIKTasks();

  std::cout << "q_lower_lim = " << robot_model->q_lower_pos_limit.transpose() << std::endl;
  std::cout << "q_upper_lim = " << robot_model->q_upper_pos_limit.transpose() << std::endl;

  feas_data_gen.randomizeStartingConfiguration();

  return 0;
}
#include <avatar_locomanipulation/helpers/NeuralNetModel.hpp>

# include <cstdlib>
# include <iostream>
# include <iomanip>
# include <cmath>

#include <avatar_locomanipulation/helpers/IOUtilities.hpp>

int main(int argc, char ** argv){
  //std::string model_path = "/home/mihir/locomanipulation_ws/src/avatar_locomanipulation/src/python_src/rh_transitions/learned_model/model.yaml";
  std::string model_path = "/home/mihir/locomanipulation_ws/src/avatar_locomanipulation/src/python_src/rh_transitions/learned_model/00050.yaml";
  std:: cout << "loading = " << model_path << std::endl;
  myYAML::Node model = myYAML::LoadFile(model_path);
  NeuralNetModel nn_transition(model["valfn_params"], false);

}

#ifndef ALM_PARAMETER_HANDLER_H
#define ALM_PARAMETER_HANDLER_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

class ParamHandler{
public:
  ParamHandler();	
  ParamHandler(const std::string & file_name);
  virtual ~ParamHandler();

  void load_yaml_file(const std::string & file_name);

  bool getString(const std::string & key, std::string & str_value);
  bool getVector(const std::string & key, std::vector<double> & vec_value);
  bool getValue(const std::string & key, double & double_value);
  bool getNestedValue(const std::vector<std::string> & nested_key, double & double_value);
  bool getBoolean(const std::string & key, bool & bool_value);
  bool getInteger(const std::string & key, int & int_value);

  YAML::Node config;
};
#endif

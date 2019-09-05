#include "myYaml/include/myYaml/null.h"

namespace myYAML {
_Null Null;

bool IsNullString(const std::string& str) {
  return str.empty() || str == "~" || str == "null" || str == "Null" ||
         str == "NULL";
}
}

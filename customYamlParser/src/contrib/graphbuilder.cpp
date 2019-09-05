#include "graphbuilderadapter.h"

#include "myYaml/include/myYaml/parser.h"  // IWYU pragma: keep

namespace myYAML {
class GraphBuilderInterface;

void* BuildGraphOfNextDocument(Parser& parser,
                               GraphBuilderInterface& graphBuilder) {
  GraphBuilderAdapter eventHandler(graphBuilder);
  if (parser.HandleNextDocument(eventHandler)) {
    return eventHandler.RootNode();
  } else {
    return NULL;
  }
}
}

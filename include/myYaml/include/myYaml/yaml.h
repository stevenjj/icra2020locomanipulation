#ifndef YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66_ASD
#define YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66_ASD

#if defined(_MSC_VER) ||                                            \
    (defined(__GNUC__) && (__GNUC__ == 3 && __GNUC_MINOR__ >= 4) || \
     (__GNUC__ >= 4))  // GCC supports "pragma once" correctly since 3.4
#pragma once
#endif

#include "myYaml/include/myYaml/parser.h"
#include "myYaml/include/myYaml/emitter.h"
#include "myYaml/include/myYaml/emitterstyle.h"
#include "myYaml/include/myYaml/stlemitter.h"
#include "myYaml/include/myYaml/exceptions.h"

#include "myYaml/include/myYaml/node/node.h"
#include "myYaml/include/myYaml/node/impl.h"
#include "myYaml/include/myYaml/node/convert.h"
#include "myYaml/include/myYaml/node/iterator.h"
#include "myYaml/include/myYaml/node/detail/impl.h"
#include "myYaml/include/myYaml/node/parse.h"
#include "myYaml/include/myYaml/node/emit.h"
#include "myYaml/include/myYaml/yaml_eigen.h"

#endif  // YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66

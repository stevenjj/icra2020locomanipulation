# Shamelessly stolen from: 
# https://gitlab.com/nasa-jsc-robotics/nasa_common_cmake/blob/137926bbdb5fe3a7ccb77d187c33e97c61b8602c/cmake/Modules/FindYamlCpp.cmake


# The file was modified to
#  1. Define cmake environment variable names to match with how yaml-cpp 
#     usually defines the include and library variables if it was built locally.
#  2. Perform a package version check
#  3. Explicitly find a shared library (libyaml.so)

# - Try to find YamlCpp
# Once done this will define
#  YamlCpp_FOUND - System has YamlCpp
#  YamlCpp_INCLUDE_DIRS - The YamlCpp include directories
#  YamlCpp_LIBRARIES - The libraries needed to use YamlCpp

#  as well renaming it to what the yaml-cpp package usually defines:

#  YAML_CPP_INCLUDE_DIR - The YamlCpp include directories 
#  YAML_CPP_LIBRARIES - The libraries needed to use YamlCpp


find_package(PkgConfig)
pkg_check_modules(PC_YAMLCPP yaml-cpp)
# message(STATUS "PKG_CONFIG_FOUND: ${PKG_CONFIG_FOUND}")
# message(STATUS "PKG_CONFIG_EXECUTABLE: ${PKG_CONFIG_EXECUTABLE}")
# message(STATUS "PKG_CONFIG_VERSION_STRING: ${PKG_CONFIG_VERSION_STRING}")
# message(STATUS "PC_YAMLCPP_FOUND: ${PC_YAMLCPP_FOUND}")
# message(STATUS "PC_YamlCpp_INCLUDE_DIRS: ${PC_YamlCpp_INCLUDE_DIRS}")
# message(STATUS "PC_YAMLCPP_LIBRARY_DIRS: ${PC_YAMLCPP_LIBRARY_DIRS}")

# YamlCpp_FIND_VERSION is the argument from the call find_package(YamlCpp version_numer)
# PC_YAMLCPP_VERSION is the version of the module if it was found using pkg_check_modules
# 	see: https://cmake.org/cmake/help/latest/module/FindPkgConfig.html

# If the version argument was not given, this sets it to version 0.0.0 as a default
if(NOT YamlCpp_FIND_VERSION)
  if(NOT YamlCpp_FIND_VERSION_MAJOR)
    set(YamlCpp_FIND_VERSION_MAJOR 0)
  endif(NOT YamlCpp_FIND_VERSION_MAJOR)
  if(NOT YamlCpp_FIND_VERSION_MINOR)
    set(YamlCpp_FIND_VERSION_MINOR 0)
  endif(NOT YamlCpp_FIND_VERSION_MINOR)
  if(NOT YamlCpp_FIND_VERSION_PATCH)
    set(YamlCpp_FIND_VERSION_PATCH 0)
  endif(NOT YamlCpp_FIND_VERSION_PATCH)

  set(YamlCpp_FIND_VERSION "${YamlCpp_FIND_VERSION_MAJOR}.${YamlCpp_FIND_VERSION_MINOR}.${YamlCpp_FIND_VERSION_PATCH}")
endif(NOT YamlCpp_FIND_VERSION)

if ( ${PC_YAMLCPP_VERSION} VERSION_LESS ${YamlCpp_FIND_VERSION} )
    message(SEND_ERROR "yaml-cpp version ${PC_YAMLCPP_VERSION} found in ${PC_YAMLCPP_LIBRARY_DIRS}, "
                   "but at least version ${YamlCpp_FIND_VERSION} is required")
	set(PC_YAMLCPP_FOUND FALSE)

else ( ${PC_YAMLCPP_VERSION} VERSION_LESS ${YamlCpp_FIND_VERSION} )

	find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
	          HINTS ${PC_YamlCpp_INCLUDE_DIRS}
	          PATH_SUFFIXES include)

	find_library(YAMLCPP_LIBRARY NAMES libyaml-cpp.so
	             HINTS ${PC_YAMLCPP_LIBRARY_DIRS})

	set(YamlCpp_LIBRARIES ${YAMLCPP_LIBRARY})
	set(YamlCpp_INCLUDE_DIRS ${YAMLCPP_INCLUDE_DIR})
	# message(STATUS "YAMLCPP_LIBRARY: ${YAMLCPP_LIBRARY}")
	# message(STATUS "YAMLCPP_INCLUDE_DIR: ${YAMLCPP_INCLUDE_DIR}")

	include(FindPackageHandleStandardArgs)
	# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE
	# if all listed variables are TRUE
	find_package_handle_standard_args(YamlCpp DEFAULT_MSG
	                                  YamlCpp_LIBRARIES YamlCpp_INCLUDE_DIRS)

	mark_as_advanced(YamlCpp_INCLUDE_DIRS YamlCpp_LIBRARIES)

	# ensure that they are cached
	SET(YamlCpp_INCLUDE_DIRS ${YamlCpp_INCLUDE_DIRS} CACHE INTERNAL "The yaml-cpp include path")
	SET(YamlCpp_LIBRARIES ${YamlCpp_LIBRARIES} CACHE INTERNAL "The libraries needed to use yaml-cpp library")

	set(YAML_CPP_INCLUDE_DIR ${YamlCpp_INCLUDE_DIRS})
	set(YAML_CPP_LIBRARIES ${YamlCpp_LIBRARIES})


endif( ${PC_YAMLCPP_VERSION} VERSION_LESS ${YamlCpp_FIND_VERSION} )


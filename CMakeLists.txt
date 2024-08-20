cmake_minimum_required(VERSION 3.1)
project(optiminf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS paraminf autojac)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)

#Find Ipopt with the help of PkgConfig
pkg_search_module(IPOPT ipopt)

# add cmake functions
list (APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include (add_doxygen_compile)
include (add_gtest_compile)

# add compile options
option(BUILD_TEST "Build tests" OFF)
option(BUILD_DOC "Build documentation" OFF)
option(BUILD_ALL "Build all" OFF)

if(BUILD_ALL)
  set(BUILD_TEST ON)
  set(BUILD_DOC ON)
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS paraminf autojac
  DEPENDS EIGEN3
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})
link_directories(${IPOPT_LIBRARY_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/data_structs/manager_container.h
  include/${PROJECT_NAME}/data_structs/nlp_data.h
  include/${PROJECT_NAME}/data_structs/calculation_result.h

  include/${PROJECT_NAME}/error_handling/optiminf_exception.h

  include/${PROJECT_NAME}/nlp_manager/nlp_manager.h

  include/${PROJECT_NAME}/setup/setup_manager_base.h

  include/${PROJECT_NAME}/preparation/preparation_manager_base.h

  include/${PROJECT_NAME}/calculation_cache/calculation_cache_manager_base.h

  include/${PROJECT_NAME}/costs/cost_base.h
  include/${PROJECT_NAME}/costs/cost_manager.h

  include/${PROJECT_NAME}/constraints/constraint_base.h
  include/${PROJECT_NAME}/constraints/constraint_manager.h
  include/${PROJECT_NAME}/constraints/constraint_bound.h

  include/${PROJECT_NAME}/splines/hermite_spline.h

  include/${PROJECT_NAME}/solver_interface/solver_interface_base.h
  include/${PROJECT_NAME}/solver_interface/solver_result_status.h
  include/${PROJECT_NAME}/solver_interface/ipopt/ipopt_interface.h

  include/${PROJECT_NAME}/export/exporter_base.h

  include/${PROJECT_NAME}/util/eigen_util.h
  include/${PROJECT_NAME}/util/parameter_naming_util.h
  include/${PROJECT_NAME}/util/file_system_util.h
)

set(SOURCES
    src/optiminf_exception.cpp

    src/calculation_result.cpp

    src/nlp_manager.cpp

    src/setup_manager_base.cpp

    src/preparation_manager_base.cpp

    src/calculation_cache_manager_base.cpp

    src/cost_manager.cpp
    src/cost_base.cpp

    src/constraint_manager.cpp
    src/constraint_base.cpp
    src/constraint_bound.cpp

    src/hermite_spline.cpp

    src/solver_interface_base.cpp
    src/solver_result_status.cpp
    src/ipopt_interface.cpp

    src/exporter_base.cpp

    src/parameter_naming_util.cpp
    src/file_system_util.cpp
)


## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${IPOPT_LIBRARIES} ${CMAKE_DL_LIBS})

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


#############
## Testing ##
#############

# header files for test classes also used in the demo application
set(TEST_CLASSES_HEADERS
  test/include/${PROJECT_NAME}/test/simple_calculation_cache.h
  test/include/${PROJECT_NAME}/test/simple_preparation_manager.h
  test/include/${PROJECT_NAME}/test/simple_constraint.h
  test/include/${PROJECT_NAME}/test/simple_cost.h
  test/include/${PROJECT_NAME}/test/simple_exporter.h
  test/include/${PROJECT_NAME}/test/unit_test_setup_manager.h
  test/include/${PROJECT_NAME}/test/dummy_solver_interface.h

  test/include/${PROJECT_NAME}/test/simple_cc_view.h
)

# source files for test classes also used in the demo application
set(TEST_CLASSES_SOURCES
  test/src/test_classes/simple_calculation_cache.cpp
  test/src/test_classes/simple_preparation_manager.cpp
  test/src/test_classes/simple_constraint.cpp
  test/src/test_classes/simple_cost.cpp
  test/src/test_classes/simple_exporter.cpp
  test/src/test_classes/dummy_solver_interface.cpp
  test/src/test_classes/unit_test_setup_manager.cpp
)

# all header files used for unit tests
set(TEST_HEADERS
    ${TEST_CLASSES_HEADERS}
)

# all source files used for unit tests
set(TEST_SOURCES
  ${TEST_CLASSES_SOURCES}

  test/src/hermite_spline_test.cpp
  test/src/util_test.cpp
  test/src/integration_test.cpp
)

add_gtest_compile(INCLUDE_PATH test/include)

#####################
## Demo Executable ##
#####################
if(NOT BUILD_TEST AND NOT CATKIN_ENABLE_TESTING)
  # define variable SOURCE_DIR if not allready defined by add_gtest_compile
  # The variable is used to find the config file for the demo application relative to the projects parent directory 
  add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
  #add_compile_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\") (reguires CMake 3.12.4)
endif()

add_executable(run-demo example/src/demo_application.cpp ${TEST_CLASSES_SOURCES} ${TEST_CLASSES_HEADERS})
target_include_directories(run-demo SYSTEM PUBLIC test/include)
set_target_properties(run-demo PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(run-demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(run-demo ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
## DOCS ##
##########

add_doxygen_compile()

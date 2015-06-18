cmake_minimum_required(VERSION 2.8.3)
project(sherpa_world_model_tf_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  tf2_msgs 
  tf2_bullet 
  tf2_ros
  tf2_kdl 
)

include_directories(include ${catkin_INCLUDE_DIRS})
MESSAGE(STATUS "INFO: used catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules") #TODO make globally available
FIND_PACKAGE(BRICS_3D REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost COMPONENTS thread)
#FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osgUtil osgViewer osgText) 
FIND_PACKAGE(Xerces) 
FIND_PACKAGE(Microblx)
ADD_DEFINITIONS(-DEIGEN3)

OPTION(USE_HDF5 "Enable HDF5 Support" ON)
IF(USE_HDF5)

    # only newer versions of FindHDF5.cmake exctract the HDF5_ROOT from the environment variable
    # - so we do it manually here
    IF("$ENV{HDF5_ROOT}" STREQUAL "")
        #nothing special here      
    ELSE("$ENV{HDF5_ROOT}" STREQUAL "")
        SET(HDF5_ROOT $ENV{HDF5_ROOT})
        SET(USE_HDF5_NON_DEFAULT_PATH TRUE) 
        MESSAGE(STATUS "INFO: HDF5_ROOT set as: $ENV{HDF5_ROOT}")
    ENDIF("$ENV{HDF5_ROOT}" STREQUAL "")
  
    FIND_PACKAGE(HDF5 REQUIRED COMPONENTS CXX HL)
    IF(USE_HDF5_NON_DEFAULT_PATH) #override results
        SET(HDF5_CXX_INCLUDE_DIR ${HDF5_ROOT}/include)        
        SET(HDF5_LIBRARIES 
            ${HDF5_ROOT}/lib/libhdf5_hl_debug.so
            ${HDF5_ROOT}/lib/libhdf5_cpp_debug.so
            ${HDF5_ROOT}/lib/libhdf5_debug.so
            #CACHE STRING "Accumulated HDF5 libraries."   
        )
    ENDIF(USE_HDF5_NON_DEFAULT_PATH)   
    MESSAGE(STATUS "INFO: used HDF5_LIBRARIES: ${HDF5_LIBRARIES}")

    IF (HDF5_FOUND)
      MESSAGE(STATUS "SUCCESSFUL: HDF5 found")
      ADD_DEFINITIONS(-DBRICS_HDF5_ENABLE)     
    ELSE (HDF5_FOUND)
      MESSAGE(SEND_ERROR "WARNING: HDF5 not found.")
    ENDIF (HDF5_FOUND)
ENDIF(USE_HDF5)

OPTION(USE_OSG "Enable Open Scene Graph" OFF)    
IF(USE_OSG)
    #FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osgUtil osgViewer osgUtil osgText)
    ADD_DEFINITIONS(-DENABLE_OSG)
ENDIF(USE_OSG)  

# add include directories (-I)
INCLUDE_DIRECTORIES(${BRICS_3D_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
        ${HDF5_CXX_INCLUDE_DIR}
)

# add library directories (-L)
LINK_DIRECTORIES(${BRICS_3D_LINK_DIRECTORIES} )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp 
  DEPENDS tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(sherpa_world_model_tf_bridge
#   src/${PROJECT_NAME}/sherpa_world_model_tf_bridge.cpp
# )

## Declare a cpp executable
# add_executable(sherpa_world_model_tf_bridge_node src/sherpa_world_model_tf_bridge_node.cpp)
add_executable(sherpa_world_model_tf_bridge 
    src/world_model_tf_bridge.cpp 
    src/RsgRosBridge.cpp
    src/RsgToTFObserver.cpp
)

set(WORLD_MODEL_LIBRARIES
        ${BRICS_3D_LIBRARIES}      
        ${XERCES_LIBRARIES}
        ${OSG_LIBRARY_DEBUG}
        ${OSGGA_LIBRARY_DEBUG}
        ${OSGVIEWER_LIBRARY_DEBUG}
        ${OSGUTIL_LIBRARY_DEBUG}
        ${OSGTEXT_LIBRARY_DEBUG}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${HDF5_LIBRARIES}
        ${UBX_LIBRARY}
	z
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sherpa_world_model_tf_bridge_node sherpa_world_model_tf_bridge_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sherpa_world_model_tf_bridge
        ${WORLD_MODEL_LIBRARIES}      
 )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sherpa_world_model_tf_bridge sherpa_world_model_tf_bridge_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sherpa_world_model_tf_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

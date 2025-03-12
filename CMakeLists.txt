cmake_minimum_required(VERSION 2.8.12)
project(larvio)

set(CMAKE_BUILD_TYPE "Release")

set( CMAKE_CXX_STANDARD 14)

# Modify cmake module path if new .cmake files are required
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf
    nav_msgs
    sensor_msgs
    geometry_msgs
    eigen_conversions
    tf_conversions
    nodelet
    image_transport
    cv_bridge
    message_filters
    pcl_conversions
    pcl_ros
    std_srvs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES system_manager estimator image_processor
    CATKIN_DEPENDS
        roscpp std_msgs tf nav_msgs sensor_msgs geometry_msgs
        eigen_conversions tf_conversions message_runtime
        image_transport cv_bridge message_filters pcl_conversions
        pcl_ros std_srvs
    DEPENDS Boost EIGEN3 OpenCV SUITESPARSE
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

# Image processor
add_library(image_processor
    src/image_processor.cpp
    src/ORBDescriptor.cpp
)
target_link_libraries(image_processor glog
    ${OpenCV_LIBRARIES}
)

# Initializer
# -- Static Initializer
add_library(staticInitializer
    src/StaticInitializer.cpp
)
target_link_libraries(staticInitializer glog
    ${SUITESPARSE_LIBRARIES}
)
# -- Dynamic Initializer
add_library(dynamicInitializer
    src/StaticInitializer.cpp
    src/DynamicInitializer.cpp
    src/feature_manager.cpp
    src/initial_alignment.cpp
    src/initial_sfm.cpp
    src/solve_5pts.cpp
)
target_link_libraries(dynamicInitializer glog
    ${SUITESPARSE_LIBRARIES}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
# -- Flexible Initializer
add_library(flexibleInitializer
    src/FlexibleInitializer.cpp
)
target_link_libraries(flexibleInitializer glog
    staticInitializer
    dynamicInitializer
)

# Estimator
add_library(estimator
    src/larvio.cpp
)
target_link_libraries(estimator glog
    flexibleInitializer
    ${SUITESPARSE_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# System Manger
add_library(system_manager
    src/System.cpp
    src/larvio.cpp
    src/image_processor.cpp
    src/feature_manager.cpp
)
add_dependencies(system_manager
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(system_manager glog
    image_processor estimator
    ${catkin_LIBRARIES}
)

# System Manager nodelet
add_library(system_manager_nodelet
    src/System_nodelet.cpp
)
add_dependencies(system_manager_nodelet
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(system_manager_nodelet glog
    system_manager
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  image_processor estimator system_manager system_manager_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*_nodelet.h" EXCLUDE
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(collision_convex_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  urdf
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_convex_model
  DEPENDS  Eigen orocos_kdl
)


link_directories(${orocos_kdl_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

add_library(collision_convex_model src/urdf_collision_parser.cpp src/distance.cpp src/gjk.cpp src/transform.cpp src/intersect.cpp src/narrowphase.cpp src/geometric_shapes.cpp)
target_link_libraries(collision_convex_model ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${OCTOMAP_LIBRARIES})

### Orocos Package Exports and Install Targets ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


cmake_minimum_required(VERSION 2.8.3)
project(rover_description)
find_package(catkin REQUIRED)

catkin_package()

# Install
set(LAUNCH_DIRS launch launch/config)
set(MESH_DIRS meshes/STL/base meshes/STL/sensors/front_lrf meshes/STL/sensors/gim30 meshes/STL/sensors/gps meshes/STL/sensors/rear_lrf)
set(URDF_DIRS robots urdf urdf/base urdf/wheel urdf/sensors/gim30 urdf/sensors/gps urdf/sensors/lrf)

foreach(dir ${LAUNCH_DIRS} ${MESH_DIRS} ${URDF_DIRS})
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


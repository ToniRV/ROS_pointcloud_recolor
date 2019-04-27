cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_recolor)

find_package(catkin_simple REQUIRED)

catkin_simple()
catkin_python_setup()

#cs_add_library(example_library src/example-library.cc)

cs_add_executable(pointcloud_recolor src/pointcloud_recolor.cpp)
#target_link_libraries(pointcloud_recolor example_library)

cs_install()

cs_install_scripts(scripts/recolor.py)

cs_export()

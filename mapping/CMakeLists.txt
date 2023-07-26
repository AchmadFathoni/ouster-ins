cmake_minimum_required(VERSION 3.8)
project(mapping)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(inertial_sense_ros REQUIRED)
find_package(ouster_ros REQUIRED)

ament_package()

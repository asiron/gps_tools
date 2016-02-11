cmake_minimum_required(VERSION 2.8)
project(gps_to_local_euclidean_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)
add_service_files(DIRECTORY srv
  FILES
  LoadGPSReference.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

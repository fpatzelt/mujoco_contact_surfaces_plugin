cmake_minimum_required(VERSION 3.0.2)

project(mujoco_contact_surfaces_plugin)
add_definitions(-std=c++17)

set( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined" )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(mujoco 2.3.3 REQUIRED)
find_package(drake REQUIRED)
find_package(Eigen3 CONFIG REQUIRED Eigen)
find_package(Boost)

add_library(${PROJECT_NAME} SHARED
  src/contact_surfaces_plugin.cc
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include  
  ${Eigen_INCLUDE_DIRS}
  ${drake_INCLUDE_DIRS}
  ${mujoco_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}  
  drake::drake
  ${mujoco_LIBRARIES}
  ${Boost_LIBRARIES}
)
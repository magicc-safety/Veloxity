cmake_minimum_required(VERSION 3.16)
project(voloxide_sil_board_shim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(VOLOXIDE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(VOLOXIDE_SIM_STATICLIB "${VOLOXIDE_REPO_ROOT}/target/debug/libsim.a")

add_custom_target(
  voloxide_sim_staticlib ALL
  COMMAND cargo build -p sim --lib
  WORKING_DIRECTORY "${VOLOXIDE_REPO_ROOT}"
  COMMENT "Building Voloxide sim static library"
  BYPRODUCTS "${VOLOXIDE_SIM_STATICLIB}"
)

add_library(voloxide_sim STATIC IMPORTED GLOBAL)
set_target_properties(voloxide_sim PROPERTIES
  IMPORTED_LOCATION "${VOLOXIDE_SIM_STATICLIB}"
)
add_dependencies(voloxide_sim voloxide_sim_staticlib)

add_executable(voloxide_sil_board src/voloxide_sil_board.cpp)
target_compile_features(voloxide_sil_board PRIVATE cxx_std_17)
target_include_directories(voloxide_sil_board PRIVATE include)
ament_target_dependencies(
  voloxide_sil_board
  rclcpp
  rosflight_msgs
  sensor_msgs
  std_srvs
)
target_link_libraries(voloxide_sil_board voloxide_sim pthread dl m)

install(TARGETS voloxide_sil_board
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

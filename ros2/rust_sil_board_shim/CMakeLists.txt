cmake_minimum_required(VERSION 3.16)
project(rust_sil_board_shim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(RUSTFLIGHT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(RUSTFLIGHT_SIM_STATICLIB "${RUSTFLIGHT_REPO_ROOT}/target/debug/libsim.a")

add_custom_target(
  rustflight_sim_staticlib ALL
  COMMAND cargo build -p sim --lib
  WORKING_DIRECTORY "${RUSTFLIGHT_REPO_ROOT}"
  COMMENT "Building RustFlight sim static library"
  BYPRODUCTS "${RUSTFLIGHT_SIM_STATICLIB}"
)

add_library(rustflight_sim STATIC IMPORTED GLOBAL)
set_target_properties(rustflight_sim PROPERTIES
  IMPORTED_LOCATION "${RUSTFLIGHT_SIM_STATICLIB}"
)
add_dependencies(rustflight_sim rustflight_sim_staticlib)

add_executable(rust_sil_board src/rust_sil_board.cpp)
target_compile_features(rust_sil_board PRIVATE cxx_std_17)
target_include_directories(rust_sil_board PRIVATE include)
ament_target_dependencies(
  rust_sil_board
  rclcpp
  rosflight_msgs
  sensor_msgs
  std_srvs
)
target_link_libraries(rust_sil_board rustflight_sim pthread dl m)

install(TARGETS rust_sil_board
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

cmake_minimum_required(VERSION 3.5)
project(go2w_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_outdoor_world_resources test/test_outdoor_world_resources.py)
endif()

ament_package()
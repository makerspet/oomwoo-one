cmake_minimum_required(VERSION 3.8)
project(oomwoo_one)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)


install(DIRECTORY rviz urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Guards the sim bumper collision-name + bridge-topic wiring (needs xacro+gz;
  # the test self-skips where they are unavailable).
  ament_add_pytest_test(test_bumper_wiring test/test_bumper_wiring.py TIMEOUT 180)
endif()

ament_package()

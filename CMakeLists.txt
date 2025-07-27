cmake_minimum_required(VERSION 3.8)
project(minim5robo_bridge)

add_compile_options(-O3)
add_compile_options(-pg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

# create component
ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/main.cpp
)
target_include_directories(${PROJECT_NAME}_component PRIVATE include)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN      "MiniM5Robo::Minim5RoboBridge"
  EXECUTABLE  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_component
  tbb
)
if(OpenCV_FOUND)
    target_include_directories(${PROJECT_NAME}_component PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBS})
endif()

install(
  DIRECTORY  launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)
# build testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

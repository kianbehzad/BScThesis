cmake_minimum_required(VERSION 3.5)
project(pack_gui)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Qt MOC
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find Qt5
if(APPLE AND EXISTS /usr/local/opt/qt)
  # Homebrew installs Qt5 (up to at least 5.9.1) in
  # /usr/local/qt5, ensure it can be found by CMake since
  # it is not in the default /usr/local prefix.
  list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt")
endif()
find_package(Qt5 COMPONENTS Core Widgets OpenGL Network REQUIRED)
list(APPEND QT5_LIBRARIES Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Network)

# include paths
include_directories(
        include
)


# interface node
add_executable(interface
        src/interface/interface_exec.cpp
#        src/interface/interface_node.cpp
        src/interface/mainwindow/mainwindow.cpp
        include/pack_gui/interface/mainwindow/mainwindow.h # for Qt MOC
        )
ament_target_dependencies(interface rclcpp)
target_link_libraries(interface ${QT5_LIBRARIES})


# install nodes
install(TARGETS
        interface
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

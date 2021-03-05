cmake_minimum_required(VERSION 3.5)
project(pack_util)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pack_msgs REQUIRED)

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


# create libgeom library
add_library(libgeom SHARED
        src/geom/angle_deg.cpp
        src/geom/circle_2d.cpp
        src/geom/line_2d.cpp
        src/geom/matrix_2d.cpp
        src/geom/polygon_2d.cpp
        src/geom/ray_2d.cpp
        src/geom/rect_2d.cpp
        src/geom/region_2d.cpp
        src/geom/sector_2d.cpp
        src/geom/segment_2d.cpp
        src/geom/size_2d.cpp
        src/geom/triangle_2d.cpp
        src/geom/vector_2d.cpp
        )
ament_target_dependencies(libgeom rclcpp pack_msgs)
ament_export_targets(export_libgeom HAS_LIBRARY_TARGET)
ament_export_libraries(libgeom)
ament_export_include_directories(include)
ament_export_dependencies(
        ament_cmake
        rclcpp
)


# create libmath library
add_library(libmath
        src/math/mathtools.cpp
        src/math/matrix.cpp
        )
ament_target_dependencies(libmath rclcpp)
target_link_libraries(libmath ${QT5_LIBRARIES} libgeom)
ament_export_targets(export_libmath HAS_LIBRARY_TARGET)
ament_export_libraries(libmath)
ament_export_include_directories(include)
ament_export_dependencies(
        ament_cmake
        rclcpp
)

# create libknowledge library
add_library(libknowledge
        src/core/knowledge.cpp
        src/core/field.cpp
        src/core/drawer.cpp
        src/core/skill_handler.cpp
        )
ament_target_dependencies(libknowledge rclcpp pack_msgs)
target_link_libraries(libknowledge ${QT5_LIBRARIES} libgeom)
ament_export_targets(export_libknowledge HAS_LIBRARY_TARGET)
ament_export_libraries(libknowledge)
ament_export_include_directories(include)
ament_export_dependencies(
        ament_cmake
        rclcpp
)


# install include directory
install(
        DIRECTORY include/
        DESTINATION include
)



# install libgeom
install(
        TARGETS libgeom
        EXPORT export_libgeom
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
# install libmath
install(
        TARGETS libmath
        EXPORT export_libmath
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
# install libknowledge
install(
        TARGETS libknowledge
        EXPORT export_libknowledge
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)


# install launch files
install(DIRECTORY tools/launch
  DESTINATION share/${PROJECT_NAME})

# install param files
install(DIRECTORY tools/params
  DESTINATION share/${PROJECT_NAME})


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

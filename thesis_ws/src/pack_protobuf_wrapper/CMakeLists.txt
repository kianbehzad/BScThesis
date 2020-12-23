cmake_minimum_required(VERSION 3.5)
project(pack_protobuf_wrapper)

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
find_package(pack_util REQUIRED)
find_package(Protobuf REQUIRED)

# find Qt5
if(APPLE AND EXISTS /usr/local/opt/qt)
  # Homebrew installs Qt5 (up to at least 5.9.1) in
  # /usr/local/qt5, ensure it can be found by CMake since
  # it is not in the default /usr/local prefix.
  list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt")
endif()
find_package(Qt5 COMPONENTS Core Widgets OpenGL Network REQUIRED)
list(APPEND QT5_LIBRARIES Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Network)


# compile proto files
set(proto_dir ${PROJECT_SOURCE_DIR}/proto/)
file(GLOB PROTOBUF_DEFINITION_FILES "${proto_dir}/*.proto")
set(PROTOBUF_INPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/proto")
set(PROTOBUF_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/include/pack_protobuf_wrapper/proto")
foreach(file ${PROTOBUF_DEFINITION_FILES})
    set(PROTOBUF_COMMAND protoc ${file})
    set(PROTOBUF_ARGUMENTS --proto_path=${PROTOBUF_INPUT_DIRECTORY} --cpp_out=${PROTOBUF_OUTPUT_DIRECTORY})
    execute_process(COMMAND ${PROTOBUF_COMMAND} ${PROTOBUF_ARGUMENTS}
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            RESULT_VARIABLE PROTOBUF_RESULT
            OUTPUT_VARIABLE PROTOBUF_OUTPUT_VARIABLE
            ERROR_VARIABLE PROTOBUF_ERROR)
endforeach()



# include paths
include_directories(
        include
        ${pack_util_INCLUDE_DIRS}
)

# grsim node
add_executable(grsim
        src/grsim/grsim_exec.cpp
        src/grsim/grsim_node.cpp
        src/common/net/udpsend.cpp
        include/pack_protobuf_wrapper/proto/grSim_Packet.pb.cc
        include/pack_protobuf_wrapper/proto/grSim_Commands.pb.cc
        include/pack_protobuf_wrapper/proto/grSim_Replacement.pb.cc
        )
ament_target_dependencies(grsim rclcpp pack_msgs pack_util)
target_link_libraries(grsim ${QT5_LIBRARIES} ${PROTOBUF_LIBRARIES})


# vision node
add_executable(vision
        src/vision/vision_exec.cpp
        src/vision/vision_node.cpp
        src/common/net/robocup_ssl_client.cpp
        src/common/net/netraw.cpp
        src/common/net/udpsend.cpp
        src/vision/convert/convert_detection.cpp
        src/vision/convert/convert_geometry.cpp
        src/vision/convert/convert_units.cpp
        include/pack_protobuf_wrapper/proto/referee.pb.cc
        include/pack_protobuf_wrapper/proto/messages_robocup_ssl_wrapper.pb.cc
        include/pack_protobuf_wrapper/proto/messages_robocup_ssl_detection.pb.cc
        include/pack_protobuf_wrapper/proto/messages_robocup_ssl_geometry.pb.cc
        include/pack_protobuf_wrapper/proto/messages_robocup_ssl_refbox_log.pb.cc
        )
ament_target_dependencies(vision rclcpp pack_msgs pack_util)
target_link_libraries(vision ${QT5_LIBRARIES} ${PROTOBUF_LIBRARIES})



# install nodes
install(TARGETS
  grsim
  vision
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

cmake_minimum_required(VERSION 3.8)
project(motor_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(motor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

set(USB_CAN_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/library/can/libusb_can.so")
if(NOT EXISTS "${USB_CAN_LIBRARY}")
  message(FATAL_ERROR "USB-CAN library not found: ${USB_CAN_LIBRARY}")
endif()

add_library(motor_driver SHARED src/motor_driver_runtime.cpp)
target_include_directories(motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library/json>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library/can>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_driver rclcpp motor_msgs)
target_link_libraries(motor_driver "${USB_CAN_LIBRARY}" Threads::Threads)
set_target_properties(motor_driver PROPERTIES INSTALL_RPATH "$ORIGIN")

add_executable(motor_driver_node src/motor_driver_node.cpp)
ament_target_dependencies(motor_driver_node rclcpp motor_msgs ament_index_cpp)
target_link_libraries(motor_driver_node motor_driver Threads::Threads)
set_target_properties(motor_driver_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/motor_driver_service.sh DESTINATION lib/${PROJECT_NAME})
install(FILES library/can/usb_can.h DESTINATION include/${PROJECT_NAME}/can)
install(FILES library/json/json.hpp DESTINATION include)
install(FILES "${USB_CAN_LIBRARY}" DESTINATION lib)
install(TARGETS motor_driver motor_driver_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp motor_msgs)

ament_package()

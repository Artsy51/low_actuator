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
find_package(rclcpp REQUIRED)
find_package(motor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(USB_CAN_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/library/can/libusb_can.so")
if(NOT EXISTS "${USB_CAN_LIBRARY}")
  message(FATAL_ERROR "USB-CAN library not found: ${USB_CAN_LIBRARY}")
endif()

add_library(usb_can SHARED IMPORTED GLOBAL)
set_target_properties(usb_can PROPERTIES
  IMPORTED_LOCATION "${USB_CAN_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/can"
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library/json>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp motor_msgs)
target_link_libraries(${PROJECT_NAME} INTERFACE usb_can)

add_executable(motor_driver_node src/motor_driver_node.cpp)
ament_target_dependencies(motor_driver_node rclcpp motor_msgs ament_index_cpp)
target_link_libraries(motor_driver_node ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES library/can/usb_can.h DESTINATION include/${PROJECT_NAME}/can)
install(FILES library/json/json.hpp DESTINATION include)
install(FILES "${USB_CAN_LIBRARY}" DESTINATION lib)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(TARGETS motor_driver_node DESTINATION lib/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp motor_msgs ament_index_cpp)

ament_package()

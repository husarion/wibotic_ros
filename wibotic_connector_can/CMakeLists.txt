cmake_minimum_required(VERSION 3.8)
project(wibotic_connector_can)
# Handle superbuild first option(USE_SUPERBUILD "Whether or not a superbuild
# should be invoked" ON)

# if(USE_SUPERBUILD) project(SUPERBUILD NONE) include(cmake/SuperBuild.cmake)
# return() else() project(wibotic_connector_can) endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake rclcpp wibotic_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_library(UAVCAN_LIB uavcan REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/platform_specific_components/linux/libuavcan/include
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/libcyphal/libuavcan/include)

add_library(wibotic_can_driver src/wibotic_can_driver.cpp)
target_compile_options(wibotic_can_driver PRIVATE -std=c++11)
target_include_directories(
  wibotic_can_driver BEFORE
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/platform_specific_components/linux/libuavcan/uavcan_linux
)
target_include_directories(
  wibotic_can_driver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(wibotic_can_driver ${PACKAGE_DEPENDENCIES})

target_link_libraries(wibotic_can_driver ${UAVCAN_LIB} rt)

add_executable(wibotic_connector_can src/wibotic_can_driver_node.cpp
                                     src/main.cpp)
target_compile_options(wibotic_connector_can PRIVATE -std=c++17)
target_include_directories(
  wibotic_connector_can
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(wibotic_connector_can ${PACKAGE_DEPENDENCIES})

target_link_libraries(wibotic_connector_can wibotic_can_driver)

install(TARGETS wibotic_connector_can wibotic_can_driver
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros_testing REQUIRED)

  # Unit tests
  ament_add_gmock(
    ${PROJECT_NAME}_test_wibotic_can_driver_node
    test/unit/test_wibotic_can_driver_node.cpp src/wibotic_can_driver_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_wibotic_can_driver_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_test_wibotic_can_driver_node
                        wibotic_can_driver)

  # Integration tests
  option(TEST_INTEGRATION "Run integration tests" ON)
  if(TEST_INTEGRATION)
    add_ros_test(test/integration/wibotic_connector_can.test.py)
  endif()

endif()

ament_package()

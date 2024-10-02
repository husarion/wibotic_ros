cmake_minimum_required(VERSION 3.10.2)
project(wibotic_connector_can)
# Handle superbuild first
# option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

# if(USE_SUPERBUILD)
#   project(SUPERBUILD NONE)
#   include(cmake/SuperBuild.cmake)
#   return()
# else()
#   project(wibotic_connector_can)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    PkgConfig
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_library(UAVCAN_LIB uavcan REQUIRED)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -Werror -std=c++11")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/platform_specific_components/linux/libuavcan/include
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcyphal/libuavcan/include
)

add_executable(wibotic_connector_can src/main.cpp)

target_include_directories(wibotic_connector_can
  BEFORE
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/platform_specific_components/linux/libuavcan/uavcan_linux
)
target_include_directories(wibotic_connector_can
  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(wibotic_connector_can ${PACKAGE_DEPENDENCIES})

target_link_libraries(wibotic_connector_can
    ${UAVCAN_LIB} rt
)

add_executable(script src/script.cpp)
target_include_directories(script
  BEFORE
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/platform_specific_components/linux/libuavcan/uavcan_linux
)
target_include_directories(script
  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(script ${PACKAGE_DEPENDENCIES})

target_link_libraries(script
    ${UAVCAN_LIB} rt
)

install(TARGETS script wibotic_connector_can DESTINATION lib/${PROJECT_NAME})


ament_package()

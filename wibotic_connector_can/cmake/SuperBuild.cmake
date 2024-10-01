# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

include(ExternalProject)

set(DEPENDENCIES
  ep_libuavcan
  ep_platform_specific_components
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ep_libuavcan/include)
ExternalProject_Add(
  ep_libuavcan
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ep_libuavcan/upstream
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  GIT_REPOSITORY https://github.com/OpenCyphal-Garage/libcyphal/
  GIT_TAG dcc3a4de237b7482e04543d2393c3a9385685312
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ep_libuavcan
  INSTALL_COMMAND make install INSTALL_PREFIX=<INSTALL_DIR>
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  )

ExternalProject_Add(
  ep_platform_specific_components
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ep_platform_specific_components/upstream
    SOURCE_SUBDIR linux/libuavcan
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY https://github.com/OpenCyphal-Garage/platform_specific_components/
    GIT_TAG 4745ef59f57b7e1c34705b127ea8c7a35e3874c1
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}

  )


ExternalProject_Add(
  ep_wibotic_connector_can
  DEPENDS ${DEPENDENCIES}
  SOURCE_DIR ${PROJECT_SOURCE_DIR}
  CMAKE_ARGS -DUSE_SUPERBUILD=OFF
  INSTALL_COMMAND ""
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

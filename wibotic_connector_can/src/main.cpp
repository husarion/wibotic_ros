// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>

#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/helpers/ostream.hpp>

#include "wibotic_connector_can/wibotic_can_driver.hpp"
#include "wibotic_connector_can/uavcan_types/wibotic/WiBoticInfo.hpp"



int main()
{
  std::cout << "Starting Wibotic CAN driver" << std::endl;
  wibotic_connector_can::WiboticCanDriver wibotic_can_driver("can0", 20, "com.wibotic.ros_connector");
  wibotic_can_driver.CreateUavCanNode();
  wibotic_can_driver.CreateWiboticInfoSubscriber();
  wibotic_can_driver.Activate();

  while (true)
  {
    wibotic_can_driver.Spin(10);
    try
    {
      std::cout << wibotic_can_driver.GetWiboticInfo() << std::endl;
    }
    catch (const std::runtime_error& e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  return 0;
}

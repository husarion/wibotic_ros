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

#ifndef WIBOTIC_CONNECTOR_CAN_WIBOTIC_CAN_DRIVER_NODE_HPP_
#define WIBOTIC_CONNECTOR_CAN_WIBOTIC_CAN_DRIVER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#define UAVCAN_CPP_VERSION UAVCAN_CPP11
#include "wibotic_connector_can/wibotic_can_driver.hpp"

namespace wibotic_connector_can
{
class WiboticCanDriverNode : public rclcpp::Node
{
public:
  WiboticCanDriverNode(const std::string& node_name,
                                             const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void DeclareParameters();
  void GetParameters();

  void CreateUavCanNode();

  std::string can_iface_name_;
  std::size_t uavcan_node_id_;
  std::string uavcan_node_name_;
  float update_time_s_;

  std::unique_ptr<WiboticCanDriver> wibotic_can_driver_;

  rclcpp::TimerBase::SharedPtr wibotic_info_timer_;

  void WiboticInfoTimerCallback();
};

}  // namespace wibotic_connector_can

#endif  // WIBOTIC_CONNECTOR_CAN_WIBOTIC_CAN_DRIVER_NODE_HPP_

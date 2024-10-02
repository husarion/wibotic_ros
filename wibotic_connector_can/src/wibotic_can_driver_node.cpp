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

#include "wibotic_connector_can/wibotic_can_driver_node.hpp"

namespace wibotic_connector_can
{
WiboticCanDriverNode::WiboticCanDriverNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) /*, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))*/
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");
  DeclareParameters();
  GetParameters();

  CreateUavCanNode();

  wibotic_info_timer_ = this->create_wall_timer(std::chrono::duration<float>(update_time_s_),
                                                std::bind(&WiboticCanDriverNode::WiboticInfoTimerCallback, this));

  // // diagnostic_updater_->setHardwareID("Battery");

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}


void WiboticCanDriverNode::DeclareParameters()
{
  this->declare_parameter("can_iface_name", "can0");
  this->declare_parameter("uavcan_node_id", 20);
  this->declare_parameter("uavcan_node_name", "com.wibotic.ros_connector");
  this->declare_parameter("update_time_s", 1.0);
}

void WiboticCanDriverNode::GetParameters()
{
  can_iface_name_ = this->get_parameter("can_iface_name").as_string();
  uavcan_node_id_ = this->get_parameter("uavcan_node_id").as_int();
  uavcan_node_name_ = this->get_parameter("uavcan_node_name").as_string();
  update_time_s_ = this->get_parameter("update_time_s").as_double();
}

void WiboticCanDriverNode::CreateUavCanNode()
{
  wibotic_can_driver_ = std::make_unique<WiboticCanDriver>(can_iface_name_, uavcan_node_id_, uavcan_node_name_);
  wibotic_can_driver_->CreateUavCanNode();
  wibotic_can_driver_->CreateWiboticInfoSubscriber();
  wibotic_can_driver_->Activate();
}

void WiboticCanDriverNode::WiboticInfoTimerCallback()
{
  if (!wibotic_can_driver_)
  {
    throw std::runtime_error("Trying to get WiboticInfo message from nonexisting driver.");
  }

  try
  {
    RCLCPP_INFO(this->get_logger(), "Getting WiboticInfo message.");
    wibotic_can_driver_->Spin(1000);
    auto wibotic_info = wibotic_can_driver_->GetWiboticInfo();
    RCLCPP_INFO_STREAM(this->get_logger(), "Got WiboticInfo message: " << std::endl << wibotic_info);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_WARN(this->get_logger(), e.what());
  }
}

}  // namespace wibotic_connector_can

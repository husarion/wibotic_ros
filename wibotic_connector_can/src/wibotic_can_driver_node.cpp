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
WiboticCanDriverNode::WiboticCanDriverNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing node.");
  DeclareParameters();
  GetParameters();

  CreateWiboticCanDriver();

  wibotic_info_pub_ = this->create_publisher<wibotic_msgs::msg::WiboticInfo>("wibotic_info", 10);

  wibotic_info_timer_ = this->create_wall_timer(
    std::chrono::duration<float>(update_time_s_),
    std::bind(&WiboticCanDriverNode::WiboticInfoTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
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

void WiboticCanDriverNode::CreateWiboticCanDriver()
{
  wibotic_can_driver_ = std::make_unique<WiboticCanDriver>(
    can_iface_name_, uavcan_node_id_, uavcan_node_name_);
  wibotic_can_driver_->CreateUavCanNode();
  wibotic_can_driver_->CreateWiboticInfoSubscriber();
  wibotic_can_driver_->Activate();
}

void WiboticCanDriverNode::WiboticInfoTimerCallback()
{
  if (!wibotic_can_driver_) {
    throw std::runtime_error("Trying to get WiboticInfo message from nonexisting driver.");
  }

  try {
    const auto update_time_ms = static_cast<std::size_t>(update_time_s_ * 1000);
    wibotic_can_driver_->Spin(update_time_ms);
    auto wibotic_info = wibotic_can_driver_->GetWiboticInfo();

    wibotic_info_pub_->publish(ConvertWiboticInfoToMsg(wibotic_info));
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(this->get_logger(), e.what());
  }
}

wibotic_msgs::msg::WiboticInfo WiboticCanDriverNode::ConvertWiboticInfoToMsg(
  const wibotic::WiBoticInfo & wibotic_info)
{
  wibotic_msgs::msg::WiboticInfo wibotic_info_msg;

  wibotic_info_msg.header.stamp = this->now();
  wibotic_info_msg.header.frame_id = "wibotic_receiver";
  wibotic_info_msg.v_mon_batt = wibotic_info.VMonBatt;
  wibotic_info_msg.i_battery = wibotic_info.IBattery;
  wibotic_info_msg.v_rect = wibotic_info.VRect;
  wibotic_info_msg.v_mon_charger = wibotic_info.VMonCharger;
  wibotic_info_msg.t_board = wibotic_info.TBoard;
  wibotic_info_msg.target_i_batt = wibotic_info.TargetIBatt;
  wibotic_info_msg.i_charger = wibotic_info.ICharger;
  wibotic_info_msg.i_single_charger2 = wibotic_info.ISingleCharger2;
  wibotic_info_msg.i_single_charger3 = wibotic_info.ISingleCharger3;

  return wibotic_info_msg;
}

}  // namespace wibotic_connector_can

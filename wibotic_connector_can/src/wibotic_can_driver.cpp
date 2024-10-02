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

#include "wibotic_connector_can/wibotic_can_driver.hpp"

namespace wibotic_connector_can
{
WiboticCanDriver::WiboticCanDriver(const std::string& can_iface_name, std::size_t node_id, const std::string& node_name)
  : can_iface_name_(can_iface_name), node_id_(node_id), node_name_(node_name)
{
}

void WiboticCanDriver::CreateUavCanNode()
{
  uavcan_node_ = uavcan_linux::makeNode({ can_iface_name_ });
  uavcan_node_->setNodeID(node_id_);
  uavcan_node_->setName(node_name_.c_str());
}

void WiboticCanDriver::CreateWiboticInfoSubscriber()
{
  wibotic_info_uavcan_sub_ = std::make_shared<uavcan::Subscriber<wibotic::WiBoticInfo>>(*uavcan_node_);
}

void WiboticCanDriver::Activate()
{
  if (!uavcan_node_)
  {
    throw std::runtime_error("Trying to activate nonexisting node.");
  }

  const int sub_res =
      wibotic_info_uavcan_sub_->start(WiBoticInfoCallbackBinder(this, &WiboticCanDriver::WiboticInfoCallback));
  if (sub_res < 0)
  {
    throw std::runtime_error("Failed to start the subscriber; error: " + std::to_string(sub_res));
  }

  const int node_start_res = uavcan_node_->start();
  if (node_start_res < 0)
  {
    throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
  }

  uavcan_node_->setModeOperational();

  activated_ = true;
}

void WiboticCanDriver::Spin(std::size_t miliseconds)
{
  if (!uavcan_node_)
  {
    throw std::runtime_error("Trying to spin nonexisting node.");
  }

  if (!wibotic_info_uavcan_sub_)
  {
    throw std::runtime_error("Trying to spin nonexisting subscriber.");
  }

  if (!activated_)
  {
    throw std::runtime_error("Trying to spin non-activated driver.");
  }

  const int res = uavcan_node_->spin(uavcan::MonotonicDuration::fromMSec(miliseconds));
  if (res < 0)
  {
    throw std::runtime_error("Failed to spin UAVCAN node, res: " + std::to_string(res));
  }
}

wibotic::WiBoticInfo WiboticCanDriver::GetWiboticInfo()
{
  if (wibotic_info_queue_.empty())
  {
    throw std::runtime_error("WiBoticInfo queue is empty.");
  }

  wibotic::WiBoticInfo wibotic_info = wibotic_info_queue_.front();
  wibotic_info_queue_.pop();
  return wibotic_info;
}

void WiboticCanDriver::WiboticInfoCallback(const wibotic::WiBoticInfo& msg)
{
  wibotic_info_queue_.push(msg);
}

}  // namespace wibotic_connector_can

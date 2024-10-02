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
#include <uavcan/equipment/air_data/TrueAirspeed.hpp>
#include <uavcan/helpers/ostream.hpp>

#include "wibotic_connector_can/wibotic_can_driver.hpp"
#include "wibotic_connector_can/uavcan_types/wibotic/WiBoticInfo.hpp"

class WiboticCanDriver : public wibotic_connector_can::WiboticCanDriverInterface
{
  typedef uavcan::MethodBinder<WiboticCanDriver*, void (WiboticCanDriver::*)(const wibotic::WiBoticInfo&)>
      WiBoticInfoCallbackBinder;

public:
  WiboticCanDriver(const std::string& can_iface_name, std::size_t node_id, const std::string& node_name)
    : can_iface_name_(can_iface_name), node_id_(node_id), node_name_(node_name)
  {
  }

  virtual void CreateUavCanNode()
  {
    uavcan_node_ = uavcan_linux::makeNode({ can_iface_name_ });
    uavcan_node_->setNodeID(node_id_);
    uavcan_node_->setName(node_name_.c_str());

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    uavcan_node_->setSoftwareVersion(sw_version);

    if (uavcan_node_->start() < 0)
    {
      throw std::runtime_error("Failed to start UAVCAN node");
    }
  }

  void Activate() override
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

  virtual void Deactivate()
  {
    DestroyUavCanNode();
    activated_ = false;
  }

  virtual void DestroyUavCanNode()
  {
    if (!uavcan_node_)
    {
      throw std::runtime_error("Trying to destroy nonexisting node.");
    }
    uavcan_node_.reset();
  }

  virtual void CreateWiboticInfoSubscriber()
  {
    wibotic_info_uavcan_sub_ = std::make_shared<uavcan::Subscriber<wibotic::WiBoticInfo>>(*uavcan_node_);
  }

  virtual void Spin(std::size_t miliseconds)
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

  wibotic::WiBoticInfo GetWiboticInfo()
  {
    if (wibotic_info_queue_.empty())
    {
      throw std::runtime_error("WiBoticInfo queue is empty.");
    }

    wibotic::WiBoticInfo wibotic_info = wibotic_info_queue_.front();
    wibotic_info_queue_.pop();
    return wibotic_info;
  }

protected:
  std::string can_iface_name_;
  std::size_t node_id_;
  std::string node_name_;
  bool activated_ = false;

  uavcan_linux::NodePtr uavcan_node_;
  std::shared_ptr<uavcan::Subscriber<wibotic::WiBoticInfo>> wibotic_info_uavcan_sub_;

  std::queue<wibotic::WiBoticInfo> wibotic_info_queue_;

  void WiboticInfoCallback(const wibotic::WiBoticInfo& msg)
  {
    wibotic_info_queue_.push(msg);
  }
};

int main()
{
  std::cout << "Starting Wibotic CAN driver" << std::endl;
  WiboticCanDriver wibotic_can_driver("can0", 20, "com.wibotic.ros_connector");
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

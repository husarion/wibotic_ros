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

#ifndef WIBOTIC_CONNECTOR_CAN_WIBOTIC_CAN_DRIVER_HPP_
#define WIBOTIC_CONNECTOR_CAN_WIBOTIC_CAN_DRIVER_HPP_

#include <memory>
#include <queue>

#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/helpers/ostream.hpp>

#include "wibotic_connector_can/uavcan_types/wibotic/WiBoticInfo.hpp"

namespace wibotic_connector_can
{

/**
 * @brief Abstract interface for the Wibotic CAN driver.
 */
class WiboticCanDriverInterface
{
public:
  /**
   * @brief Virtual destructor for the WiboticCanDriverInterface class.
   */
  virtual ~WiboticCanDriverInterface() = default;

  /**
   * @brief Creates the UAVCAN node.
   */
  virtual void CreateUavCanNode() = 0;

  /**
   * @brief Creates the WiboticInfo subscriber.
   */
  virtual void CreateWiboticInfoSubscriber() = 0;

  /**
   * @brief Activates the Wibotic CAN driver.
   *
   */
  virtual void Activate() = 0;

  /**
   * @brief Spins the Wibotic CAN driver.
   *
   * @param microseconds The time to spin in microseconds.
   */
  virtual void Spin(std::size_t microseconds) = 0;

  /**
   * @brief Gets the WiboticInfo message.
   *
   * @return The WiboticInfo message.
   */
  virtual wibotic::WiBoticInfo GetWiboticInfo() = 0;

  /**
   * @brief Alias for a shared pointer to a WiboticCanDriverInterface object.
   */
  using SharedPtr = std::shared_ptr<WiboticCanDriverInterface>;

  /**
   * @brief Alias for a unique pointer to a WiboticCanDriverInterface object.
   */
  using UniquePtr = std::unique_ptr<WiboticCanDriverInterface>;
};

/**
 * @brief Class for the Wibotic CAN driver.
 *
 * This class inherits from the `WiboticCanDriverInterface` and implements its methods.
 * Class communicates with CAN interface and gets WiboticInfo messages.
 */
class WiboticCanDriver : public WiboticCanDriverInterface
{
  typedef uavcan::MethodBinder<WiboticCanDriver*, void (WiboticCanDriver::*)(const wibotic::WiBoticInfo&)>
      WiBoticInfoCallbackBinder;

public:
  /**
   * @brief Constructor for the WiboticCanDriver class.
   *
   * @param can_iface_name The name of the CAN interface.
   * @param node_id The ID of the node.
   * @param node_name The name of the node.
   *
   * @exception std::runtime_error Thrown if can interface cannot be found.
   */
  WiboticCanDriver(const std::string& can_iface_name, std::size_t node_id, const std::string& node_name);

  /**
   * @brief Creates the UAVCAN node.
   */
  void CreateUavCanNode() override;


  void CreateWiboticInfoSubscriber() override;

  /**
   * @brief Activates the Wibotic CAN driver.
   *
   * It starts the UAVCAN node, sets it to operational mode and starts Wibotic subscriber.
   *
   * @exception std::runtime_error Thrown if the node or subscriber does not exist and they does not start properly.
   */
  void Activate() override;

  /**
   * @brief Spins the Wibotic CAN driver.
   *
   * @param microseconds The time to spin in microseconds.
   *
   * @exception std::runtime_error Thrown if the node or subscriber does not spin properly.
   */
  void Spin(std::size_t microseconds) override;

  /**
   * @brief Gets the WiboticInfo message.
   *
   * @return The WiboticInfo message.
   *
   * @exception std::runtime_error Thrown if the WiboticInfo message queue is empty.
   */
  wibotic::WiBoticInfo GetWiboticInfo() override;

protected:
  /**
   * @brief Callback for the WiboticInfo message.
   *
   * Adds uavcan messages to the queue.
   *
   * @param msg The WiboticInfo message.
   */
  void WiboticInfoCallback(const wibotic::WiBoticInfo& msg);

  std::string can_iface_name_;
  std::size_t node_id_;
  std::string node_name_;
  bool activated_ = false;

  uavcan_linux::NodePtr uavcan_node_;
  std::shared_ptr<uavcan::Subscriber<wibotic::WiBoticInfo>> wibotic_info_uavcan_sub_;

  std::queue<wibotic::WiBoticInfo> wibotic_info_queue_;
};

}  // namespace wibotic_connector_can

#endif  // WIBOTIC_CONNECTOR_CAN_WIBOTIC_CAN_DRIVER_HPP_

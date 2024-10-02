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

#ifndef WIBOTIC_CONNECTOR_CAN_WIBOTIC_CONNECTOR_CAN_HPP_
#define WIBOTIC_CONNECTOR_CAN_WIBOTIC_CONNECTOR_CAN_HPP_

#include <memory>

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

  virtual void Activate() = 0;
  virtual void Deactivate() = 0;

  virtual void CreateUavCanNode() = 0;
  virtual void DestroyUavCanNode() = 0;

  virtual void CreateWiboticInfoSubscriber() = 0;

  virtual void Spin(std::size_t microseconds) = 0;

  /**
   * @brief Alias for a shared pointer to a WiboticCanDriverInterface object.
   */
  using SharedPtr = std::shared_ptr<WiboticCanDriverInterface>;

  /**
   * @brief Alias for a unique pointer to a WiboticCanDriverInterface object.
   */
  using UniquePtr = std::unique_ptr<WiboticCanDriverInterface>;

protected:

};





}  // namespace wibotic_connector_can

#endif  // WIBOTIC_CONNECTOR_CAN_WIBOTIC_CONNECTOR_CAN_HPP_

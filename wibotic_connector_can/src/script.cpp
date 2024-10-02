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

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/equipment/power/BatteryInfo.hpp>

#include "wibotic_connector_can/wibotic_can_driver.hpp"
#include "wibotic_connector_can/uavcan_types/wibotic/WiBoticInfo.hpp"

#include <uavcan_linux/uavcan_linux.hpp>

uavcan::ISystemClock& getSystemClock()
{
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    if (driver.getNumIfaces() == 0)     // Will be executed once
    {
        if (driver.addIface("can0") < 0)
        {
            throw std::runtime_error("Failed to add iface");
        }
    }
    return driver;
}


// constexpr unsigned NodeMemoryPoolSize = 16384;
// typedef uavcan::Node<NodeMemoryPoolSize> Node;


// static Node& getNode()
// {
//     static Node node(getCanDriver(), getSystemClock());
//     return node;
// }
/**
 * This class demonstrates how to use uavcan::MethodBinder with subscriber objects in C++03.
 * In C++11 and newer standards it is recommended to use lambdas and std::function<> instead, as this approach
 * would be much easier to implement and to understand.
 */
class Node
{
    static const unsigned NodeMemoryPoolSize = 16384;

    uavcan::Node<NodeMemoryPoolSize> node_;

    /*
     * Instantiations of uavcan::MethodBinder<>
     */
    typedef uavcan::MethodBinder<Node*, void (Node::*)(const wibotic::WiBoticInfo&) const>
        LogMessageCallbackBinder;



    uavcan::Subscriber<wibotic::WiBoticInfo, LogMessageCallbackBinder> log_sub_;


    void logMessageCallback(const wibotic::WiBoticInfo& msg) const
    {
        std::cout << "Log message:\n" << msg << std::endl;
    }



public:
    Node(uavcan::NodeID self_node_id, const std::string& self_node_name) :
        node_(getCanDriver(), getSystemClock()),
        log_sub_(node_)
    {
        node_.setNodeID(self_node_id);
        node_.setName(self_node_name.c_str());
    }

    void run()
    {
        const int start_res = node_.start();
        if (start_res < 0)
        {  std::cout << "Starting Wibotic CAN driver" << std::endl;

            throw std::runtime_error("Failed to start the node: " + std::to_string(start_res));
        }

        const int log_sub_start_res = log_sub_.start(LogMessageCallbackBinder(this, &Node::logMessageCallback));
        if (log_sub_start_res < 0)
        {
            throw std::runtime_error("Failed to start the log subscriber; error: " + std::to_string(log_sub_start_res));
        }
        node_.setModeOperational();

        while (true)
        {
            const int res = node_.spin(uavcan::MonotonicDuration::fromMSec(1000));
            if (res < 0)
            {
                std::cerr << "Transient failure: " << res << std::endl;
            }

                std::cout << "spinnging... "<< std::endl;

        }
    }
};

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <node-id>" << std::endl;
        return 1;
    }

  std::cout << "Starting Wibotic CAN driver SScript not class" << std::endl;

    const int self_node_id = std::stoi(argv[1]);

    Node node(self_node_id, "org.uavcan.tutorial.subscriber_cpp03");

    node.run();
}

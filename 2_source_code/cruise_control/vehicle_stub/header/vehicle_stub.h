/*
*   @file vehicle_stub.h
*	@brief This is file declears functions required for vehicle stub
*
*	1. Declares the ros mesaage type to be used for publishing and subscribing.
*	2. Declares fuctions to create and publish the vehicle speed based on subscribed
*      acceleration.
*/
#ifndef CRUISE_CONTROL_VEHICLE_STUB_HEADER_VEHICLE_STUB_H
#define CRUISE_CONTROL_VEHICLE_STUB_HEADER_VEHICLE_STUB_H

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "../../ros/header/publisher.h"

namespace cruise_control
{
    
using ros_msg_type = std_msgs::msg::Float32;

class VehicleStub
{
public:
    VehicleStub() = default;
    ~VehicleStub() = default;
    VehicleStub(const VehicleStub&) = delete;
    VehicleStub& operator=(const VehicleStub&) = delete;
    VehicleStub(const VehicleStub&&) = delete;
    VehicleStub& operator=(const VehicleStub&&) = delete;		

    void ExecuteVehicleStubManager(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> &future);

private:
    std::shared_ptr < Publisher<ros_msg_type>> CreatePublisher(std::shared_ptr<rclcpp::Node> node);

    void PublishVehicleSpeed(std::shared_ptr<rclcpp::Node> node,
                                    std::shared_ptr< Publisher<ros_msg_type>> publisher,
                                    std::shared_future<void> &future);
};
} //namespace cruise_control
#endif //CRUISE_CONTROL_VEHICLE_STUB_HEADER_STUB_NODE_H

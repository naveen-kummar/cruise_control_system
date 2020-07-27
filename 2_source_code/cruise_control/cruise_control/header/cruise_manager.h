/*
*   @file cruise_manager.h
*	@brief Declares the function to create and publish the acceleration based on
*   received vehicle speed from vehicle stub node.
*/
#ifndef CRUISE_CONTROL_CRUISE_CONTROL_HEADER_CRUISE_MANAGER_H
#define CRUISE_CONTROL_CRUISE_CONTROL_HEADER_CRUISE_MANAGER_H
#include <string>
#include <memory>
#include <future>

#include "../../configuration/header/cruise_configuration.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "../../ros/header/publisher.h"


namespace cruise_control
{
constexpr int32_t kInvalidSetSpeed{-1};

using ros_msg_type = std_msgs::msg::Float32;

class CruiseManager
{
public:
    CruiseManager() = default;
    ~CruiseManager() = default;
    CruiseManager(const CruiseManager&) = delete;
    CruiseManager& operator=(const CruiseManager&) = delete;
    CruiseManager(const CruiseManager&&) = delete;
    CruiseManager& operator=(const CruiseManager&&) = delete;		

    void ExecuteCruiseControl(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> &future,
                                        int32_t set_speed);

private:
    std::shared_ptr < Publisher<ros_msg_type>> CreatePublisher(std::shared_ptr<rclcpp::Node> node);

    void PublishRequiredAcceleration(std::shared_ptr<CruiseConfiguration> config_manager,
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr< Publisher<ros_msg_type>> publisher,
        std::shared_future<void> &future);

    void ShowAndUpdateConfigParams(std::shared_ptr<CruiseConfiguration> config_manager);

    void UpdateCruiseParameters(cruise_param &config_list);
};
} //namespace cruise_control
#endif //CRUISE_CONTROL_CRUISE_CONTROL_HEADER_CRUISE_MANAGER_H

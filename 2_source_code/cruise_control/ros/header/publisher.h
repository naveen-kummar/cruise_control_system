/*
*   @file publisher.h
*	@brief This file declares class to abstract the ROS2 node's publisher
*
*	Defines function (templatized for ros message type) for starting 
*   publisher loop and publisher call back function.
*/
#ifndef CRUISE_CONTROL_ROS_HEADER_PUBLISHER_H
#define CRUISE_CONTROL_ROS_HEADER_PUBLISHER_H
#include <iostream>
#include <functional>
#include <memory>
#include <mutex>

#include "topic_info.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace cruise_control
{
using namespace std::chrono_literals;

template <typename T>
class Publisher
{
public:
    Publisher(const TopicInfo& topic,
        std::shared_ptr<rclcpp::Node> node,
        std::chrono::milliseconds freq) :
        topic_data_{ topic }, node_{ node }, frequency_{ freq }, value_{ std::make_shared<T>() }
    {
        bPublishingStarted_ = false;
    }
    ~Publisher() = default;
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;
    Publisher(const Publisher&&) = delete;
    Publisher& operator=(const Publisher&&) = delete;		

    bool PublishValue(T value)
    {
        bool is_published{true};

        try
        {
            std::lock_guard<std::mutex> lock_data(publisher_mutex);
            value_->data = value.data;
            if (bPublishingStarted_ == false)
            {
                bPublishingStarted_ = true;
                publisher_ = node_->create_publisher<T>(topic_data_.GetTopicName(),
                    topic_data_.GetQueueSize());

                timer_ = node_->create_wall_timer(
                    frequency_, std::bind(&Publisher::topic_callback, this));

            }
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error occurred while publishing = "<< e.what() << '\n';
            is_published = false;
        }

        return is_published;

    }

private:
    std::mutex publisher_mutex{};	

    const TopicInfo topic_data_;

    std::shared_ptr<rclcpp::Node> node_;

    const std::chrono::milliseconds frequency_;

    typename T::SharedPtr value_;

    typename rclcpp::Publisher<T>::SharedPtr publisher_;		

    rclcpp::TimerBase::SharedPtr timer_;

    bool bPublishingStarted_;

    void topic_callback()
    {
        std::lock_guard<std::mutex> lock_data(publisher_mutex);
        RCLCPP_INFO(node_->get_logger(), "Publishing: '%f'", value_->data);
        publisher_->publish(*value_);
    }
};
} // namespace cruise_control
#endif // CRUISE_CONTROL_ROS_HEADER_PUBLISHER_H

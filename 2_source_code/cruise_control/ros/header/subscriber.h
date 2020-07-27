/*
*   @file subscriber.h
*	@brief This file declares class to abstract the ROS2 node's subscriber
*
*	Defines function (templatized for ros message type) for starting
*   subscriber loop and subscriber call back function.
*/
#ifndef CRUISE_CONTROL_ROS_HEADER_SUBSCRIBER_H
#define CRUISE_CONTROL_ROS_HEADER_SUBSCRIBER_H
#include <iostream>
#include <functional>
#include <memory>
#include <mutex>

#include "topic_info.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace cruise_control
{
    
using std::placeholders::_1;

template <typename T>
class Subscriber
{
public:
    Subscriber(const TopicInfo& topic,
        std::shared_ptr<rclcpp::Node> node) :
        topic_data_{ topic }, node_{ node }, value_{ std::make_shared<T>() }
    {
    }
    ~Subscriber() = default;
    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;
    Subscriber(const Subscriber&&) = delete;
    Subscriber& operator=(const Subscriber&&) = delete;			

    bool StartSubscription()
    {
        bool is_subscribed{true};
        try
        {
            subscription_ = node_->create_subscription<T>(
                topic_data_.GetTopicName(), topic_data_.GetQueueSize(),
                std::bind(&Subscriber::topic_callback, this, _1));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Subscription creation failed" << e.what() << '\n';
            is_subscribed = false;
        }

        return is_subscribed;
    }

    typename T::SharedPtr GetValue()
    {
        std::lock_guard<std::mutex> lock_data(subscriber_mutex);
        return value_;
    }

    void ResetValue(T reset_value)
    {
        std::lock_guard<std::mutex> lock_data(subscriber_mutex);
        value_->data = reset_value.data;
        return;
    }


private:
    std::mutex subscriber_mutex{};

    const TopicInfo topic_data_;

    std::shared_ptr<rclcpp::Node> node_;

    typename rclcpp::Subscription<T>::SharedPtr subscription_;

    typename  T::SharedPtr value_;

    void topic_callback(typename T::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Subscribed value: '%f'", msg->data);

        std::lock_guard<std::mutex> lock_data(subscriber_mutex);
        value_->data = msg->data;
    }
};
} //namespace cruise_control
#endif // CRUISE_CONTROL_ROS_HEADER_SUBSCRIBER_H

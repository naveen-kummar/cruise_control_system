/*
*   @file topic_info.h
*	@brief This file define class for storing topic data information
*
*	Declares and define functionality to retrive the topic information
*   which is assigned while creating object of this class
*/
#ifndef CRUISE_CONTROL_ROS_HEADER_TOPIC_INFO_H
#define CRUISE_CONTROL_ROS_HEADER_TOPIC_INFO_H
#include <string>
#include <chrono>

namespace cruise_control
{
using namespace std::chrono_literals;

constexpr int kTopicBufferLength{10};
constexpr char kTopicAcceleration[]{"acceleration"};
constexpr char kTopicVehicleSpeed[]{"vehicle_speed"};
constexpr std::chrono::milliseconds kTopicPeriod{500ms};
constexpr std::chrono::seconds KThreadExecutionFrequency{1s};

class TopicInfo
{
public:
    TopicInfo(const std::string name, const std::size_t size) :
        topic_name_{ name }, queue_size_{ size }
    {

    }
    ~TopicInfo() = default;
    TopicInfo(const TopicInfo&) = default;
    TopicInfo& operator=(const TopicInfo&) = default;
    TopicInfo(const TopicInfo&&) = delete;
    TopicInfo& operator=(const TopicInfo&&) = delete;

    inline std::string GetTopicName() const
    {
        return topic_name_;
    }

    inline std::size_t GetQueueSize() const
    {
        return queue_size_;
    }

private:
    std::string  topic_name_;
    std::size_t queue_size_;

};
} //namespace cruise_control
#endif // CRUISE_CONTROL_ROS_HEADER_TOPIC_INFO_H

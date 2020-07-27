/*
*   @file vehicle_stub.cpp
*	@brief This is file implements vehicle stub functionality
*
*	Responsible for subscribing acceleration and publishing updated
*	vehicle speed through a custom templatized publisher and subscriber
*	which abstracts ROS2 node.
*/
#include <memory>
#include <iostream>

#include "../header/vehicle_stub.h"
#include "../../ros/header/topic_info.h"
#include "../../ros/header/subscriber.h"
#include "../../algorithm/header/cruise_algo_factory.h"

namespace cruise_control
{
    
constexpr float kZeroAcceleration{0.0F};
constexpr float kZeroSpeed{0.0F};
constexpr float kResistanceDrag{0.25F};

void VehicleStub::ExecuteVehicleStubManager(std::shared_ptr<rclcpp::Node> node,
                                            std::shared_future<void> &future)
{
    //Create publisher for acceleration using node
    std::shared_ptr< Publisher<ros_msg_type>> speed_publisher =
        CreatePublisher(node);

    //Publish acceleration using algo..
    PublishVehicleSpeed(node, speed_publisher, future);
}

std::shared_ptr< Publisher<ros_msg_type>>  VehicleStub::CreatePublisher(std::shared_ptr<rclcpp::Node> node)
{
    //Create topic for publishing acceleration
    const TopicInfo topic_current_speed(kTopicVehicleSpeed, kTopicBufferLength);

    return std::make_shared<Publisher<ros_msg_type>>(topic_current_speed, node, kTopicPeriod);
}

void  VehicleStub::PublishVehicleSpeed(std::shared_ptr<rclcpp::Node> node,
                                std::shared_ptr< Publisher<ros_msg_type>> publisher,
                                std::shared_future<void> &future)
{
    std_msgs::msg::Float32 speed_output{};

    //Create topic for current vehicle speed 
    const TopicInfo topic_acceleration(kTopicAcceleration, kTopicBufferLength);

    //Create subscriber for above topic
    std::shared_ptr< Subscriber<ros_msg_type>> acceleration_subscriber =
        std::make_shared<Subscriber<ros_msg_type>>(topic_acceleration, node);

    //Start the subscription loop
    if(!acceleration_subscriber->StartSubscription())
    {
        return;
    }

    std_msgs::msg::Float32 publish_speed{};
    std_msgs::msg::Float32 reset_acceleration{};

    while (future.wait_for(std::chrono::seconds(KThreadExecutionFrequency)) 
                                                    == 	std::future_status::timeout)
    {		
        const float current_acceleration = (acceleration_subscriber->GetValue())->data;

        if (current_acceleration > kZeroAcceleration)
        {
            publish_speed.data = publish_speed.data + current_acceleration;
            acceleration_subscriber->ResetValue(reset_acceleration);
        }
        else
        {
            publish_speed.data -= kResistanceDrag;
            if(publish_speed.data < kZeroSpeed)
            {
                publish_speed.data = kZeroSpeed;
            }
        }
        
        if(publisher->PublishValue(publish_speed) == false)
        {
            break;
        }

    }

    return;
}

} //namespace cruise_control

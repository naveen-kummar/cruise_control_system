/*
*   @file cruise_manager.cpp
*	@brief This file implements the cruise control functionallities
*
*	This file responsible for below items
*	1. Create Configuration Manager setting cruise parameters like SET_SPEED
*   2. Use Factory method to get Algorithm object based on parameters configured
*   3. Calculates required acceleration based on received vehicle speed
*   4. Publishes the acceleration to the vehicle stub.
*/
#include <memory>
#include <iostream>

#include "../header/cruise_manager.h"
#include "../header/units.h"
#include "../../ros/header/topic_info.h"
#include "../../ros/header/subscriber.h"
#include "../../algorithm/header/cruise_algo_factory.h"

using namespace units;

namespace cruise_control
{

void CruiseManager::ExecuteCruiseControl(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> &future,
                                        int32_t set_speed)
{
    //Read cruise configuration data
    std::shared_ptr<CruiseConfiguration> cruise_configuration =
        std::make_shared<CruiseConfiguration>();
        
    cruise_configuration->SetDefaultValues();

    if(set_speed != kInvalidSetSpeed)
    {
        cruise_configuration->SetParamValue(kParamSetSpeed, set_speed);
    }

    //return if no cruise configuration setting done
    if (cruise_configuration->GetCount() == 0)
        return;

    ShowAndUpdateConfigParams(cruise_configuration);

    //Create publisher for acceleration using node
    std::shared_ptr< Publisher<ros_msg_type>> accelerator_publisher =
                                                        CreatePublisher(node);

    //Publish acceleration using algo..
    PublishRequiredAcceleration(cruise_configuration, node, accelerator_publisher, future);

}

std::shared_ptr< Publisher<ros_msg_type>>  CruiseManager::CreatePublisher(std::shared_ptr<rclcpp::Node> node)
{
    //Create topic for publishing acceleration
    const TopicInfo topic_acceleration(kTopicAcceleration, kTopicBufferLength);

    return std::make_shared<Publisher<ros_msg_type>>(topic_acceleration, node, kTopicPeriod);
}

void  CruiseManager::PublishRequiredAcceleration(std::shared_ptr<CruiseConfiguration> cruise_configuration,
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr< Publisher<ros_msg_type>> publisher,
    std::shared_future<void> &future)
{
    std_msgs::msg::Float32 acceleration_output{};

    //Check configured set speed available 
    if (cruise_configuration->IsParamExist(kParamSetSpeed) != 0)
    {
        double set_speed_value = cruise_configuration->GetParamValue(kParamSetSpeed);
        const units::velocity::kilometers_per_hour_t set_speed{set_speed_value};

        //Create topic for current vehicle speed 
        const TopicInfo topic_current_speed(kTopicVehicleSpeed, kTopicBufferLength);

        //Create subscriber for above topic
        std::shared_ptr< Subscriber<ros_msg_type>> speed_subscriber =
            std::make_shared<Subscriber<ros_msg_type>>(topic_current_speed, node);

        //Start the subscription loop
        if(speed_subscriber->StartSubscription() == false)
        {
            return;
        }

        //Create algorithim object using strategy + Decorator pattern
        std::unique_ptr<ICruiseStrategy> cruise_algo
            = CruiseAlgoFactory::CreateCruiseAlgo(cruise_configuration);

        while (future.wait_for(std::chrono::seconds(KThreadExecutionFrequency)) 
                                                    == 	std::future_status::timeout)
        {
            const units::velocity::kilometers_per_hour_t 
                            current_speed{speed_subscriber->GetValue()->data};

            if (current_speed != set_speed)
            {
                acceleration_output.data =
                    unit_cast<float>(cruise_algo->GetAcceleration(current_speed));

                if(publisher->PublishValue(acceleration_output) == false)
                {
                    break;
                }

            }

        }

    }

    return;
}

void CruiseManager::ShowAndUpdateConfigParams(std::shared_ptr<CruiseConfiguration> cruise_configuration)
{
    std::cout << "\\\\\\\\Currently Configured Default Cruise Values///////////" << std::endl;

    //GetConfigList

    auto &config_list{cruise_configuration->GetConfigList()};

    int32_t params{0};
    for(auto config : config_list)
    {
        std::cout << ++params << ". " << config.first << " = " << config.second << std::endl;
    }

    std::cout << std::endl;

    std::cout << "Want to change above values? (Y)" << std::endl;

    char input;

    std::cin >> input;

    if(input == 'y' || input == 'Y')
    {
        UpdateCruiseParameters(config_list);

        std::cout << "<<<<<<<<<<<<<<<<<<<New Updated Values are below>>>>>>>>>>>>>>>>>" << std::endl;
        
        params = 0;
        for(auto config : config_list)
        {
            std::cout << ++params << ". " << config.first << " = " << config.second << std::endl;
        }

    }


}

void CruiseManager::UpdateCruiseParameters(cruise_param &config_list)
{
    int32_t params{0};
    for(auto &config : config_list)
    {
        std::cout << ++params << ". " << config.first << " = " << config.second << std::endl;
        std::cout << "Change above value? (Y)" << std::endl;
        char input;
        std::cin >> input;

        if(input == 'y' || input == 'Y')
        {
            float value;
            std::cin >> value;
            config.second = value;

        }

    }
}

} //namespace cruise_control

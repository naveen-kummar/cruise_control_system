/*
*   @file cruise_node_runner.cpp
*	@brief This file creates and manages threads to execute Ros node and
*   Cruise Manager.
*   Also take care of exiting a thread if other thread is exited
*   
*/
#include "../header/cruise_node_runner.h"

constexpr char kCruiseControlNode[] = "cruise_control_node";

namespace cruise_control
{

std::promise<void> CruiseNodeRunner::exit_signal_{std::promise<void>{}};
bool CruiseNodeRunner::promise_set_{false};
std::mutex CruiseNodeRunner::promise_mutex_{};

void CruiseNodeRunner::Run(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    //Check we received SET_SPEED from ros launch command
    int32_t set_speed{(argc > 1)? (std::atoi(argv[1])) : kInvalidSetSpeed};

    auto node = rclcpp::Node::make_shared(kCruiseControlNode);

    std::shared_future<void> future_object = exit_signal_.get_future();

    std::thread cruise_control_thread(ExecuteCruiseControl, node, future_object, set_speed);
    std::thread cruise_node_thread(ExecuteCruiseNode, node, future_object);

    cruise_control_thread.join();
    cruise_node_thread.join();
}

void CruiseNodeRunner::ExecuteCruiseNode(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> future)
                                        
{
    try
    {	
        rclcpp::spin_until_future_complete(node, future);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Ros spin error = " << e.what() << '\n';

    }

    SignalPromise();
}

void CruiseNodeRunner::ExecuteCruiseControl(std::shared_ptr<rclcpp::Node> node,
                                            std::shared_future<void> future,
                                            int32_t set_speed)
{
    std::unique_ptr<cruise_control::CruiseManager> cruise_manager = std::make_unique<cruise_control::CruiseManager>();

    try
    {
        cruise_manager->ExecuteCruiseControl(node, future, set_speed);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Cruise control execution error = "<< e.what() << '\n';
    }

    SignalPromise();
}

void CruiseNodeRunner::SignalPromise()
{
    std::lock_guard<std::mutex> lock_data(promise_mutex_);
    if(!promise_set_)
    {
        exit_signal_.set_value();
        promise_set_ = true;
    }
}



} //namespace cruise_control

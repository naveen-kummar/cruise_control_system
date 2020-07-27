/*
*   @file vehicle_stub_runner.cpp
*	@brief This file creates and manages threads to execute Ros node and
*   Cruise Manager.
*   Also take care of exiting a thread if other thread is exited
*/
#include "../header/vehicle_stub_runner.h"

constexpr char kVehicleNodeName[] = "vehicle_stub_node";

namespace cruise_control
{

std::promise<void> VehicleStubRunner::exit_signal_{std::promise<void>{}};
bool VehicleStubRunner::promise_set_{false};
std::mutex VehicleStubRunner::promise_mutex_{};

void VehicleStubRunner::Run(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(kVehicleNodeName);

    std::shared_future<void> future_object = exit_signal_.get_future();

    std::thread vehicle_stub_thread(ExecuteVehicleStub, node, future_object);
    std::thread vehicle_node_thread(ExecuteVehicleNode, node, future_object);

    vehicle_stub_thread.join();
    vehicle_node_thread.join();
}

void VehicleStubRunner::ExecuteVehicleNode(std::shared_ptr<rclcpp::Node> node,
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

void VehicleStubRunner::ExecuteVehicleStub(std::shared_ptr<rclcpp::Node> node,
                                            std::shared_future<void> future)
{
    std::unique_ptr<cruise_control::VehicleStub> vehicle_stub = 
                            std::make_unique<cruise_control::VehicleStub>();

    try
    {
        vehicle_stub->ExecuteVehicleStubManager(node, future);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Vehicle Stub execution error = "<< e.what() << '\n';
    }

    SignalPromise();
}

void VehicleStubRunner::SignalPromise()
{
    std::lock_guard<std::mutex> lock_data(promise_mutex_);
    if(!promise_set_)
    {
        exit_signal_.set_value();
        promise_set_ = true;
    }
}

} //namespace cruise_control

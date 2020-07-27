/*
*   @file vehicle_stub_runner.h
*	@brief Declares the function to create and execute the thread for Ros node
*   and Vehicle Stub
*/
#ifndef CRUISE_CONTROL_CRUISE_CONTROL_HEADER_VEHICLE_STUB_RUNNER_H
#define CRUISE_CONTROL_CRUISE_CONTROL_HEADER_VEHICLE_STUB_RUNNER_H
#include <memory>
#include <thread>
#include <future>
#include <mutex>

#include "../header/vehicle_stub.h"


namespace cruise_control
{

class VehicleStubRunner
{
public:
    VehicleStubRunner() = default;
    ~VehicleStubRunner() = default;
    VehicleStubRunner(const VehicleStubRunner&) = delete;
    VehicleStubRunner& operator=(const VehicleStubRunner&) = delete;
    VehicleStubRunner(const VehicleStubRunner&&) = delete;
    VehicleStubRunner& operator=(const VehicleStubRunner&&) = delete;		

    void static Run(int argc, char* argv[]);

private:
    static std::promise<void> exit_signal_;

    static bool promise_set_;

    static std::mutex promise_mutex_;

    void static ExecuteVehicleNode(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> future);

    void static ExecuteVehicleStub(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> future);

    void static SignalPromise();
};
} //namespace cruise_control
#endif //CRUISE_CONTROL_CRUISE_CONTROL_HEADER_VEHICLE_STUB_RUNNER_H

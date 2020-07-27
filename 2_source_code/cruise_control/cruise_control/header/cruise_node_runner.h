/*
*   @file cruise_node_runner.h
*	@brief Declares the function to create and execute the thread for Ros node
*   and Cruise Manager
*/
#ifndef CRUISE_CONTROL_CRUISE_CONTROL_HEADER_CRUISE_NODE_RUNNER_H
#define CRUISE_CONTROL_CRUISE_CONTROL_HEADER_CRUISE_NODE_RUNNER_H
#include <memory>
#include <thread>
#include <future>
#include <mutex>

#include "../header/cruise_manager.h"


namespace cruise_control
{

class CruiseNodeRunner
{
public:
    CruiseNodeRunner() = default;
    ~CruiseNodeRunner() = default;
    CruiseNodeRunner(const CruiseNodeRunner&) = delete;
    CruiseNodeRunner& operator=(const CruiseNodeRunner&) = delete;
    CruiseNodeRunner(const CruiseNodeRunner&&) = delete;
    CruiseNodeRunner& operator=(const CruiseNodeRunner&&) = delete;		

    void static Run(int argc, char* argv[]);

private:
    static std::promise<void> exit_signal_;

    static bool promise_set_;

    static std::mutex promise_mutex_;

    void static ExecuteCruiseNode(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> future);

    void static ExecuteCruiseControl(std::shared_ptr<rclcpp::Node> node,
                                        std::shared_future<void> future,
                                        int32_t set_speed);

    void static SignalPromise();
};
} //namespace cruise_control
#endif //CRUISE_CONTROL_CRUISE_CONTROL_HEADER_CRUISE_NODE_RUNNER_H

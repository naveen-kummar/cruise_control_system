/*
*   @file basic_cruise_control.h
*	@brief Declares basic cruise control algorithm
*/
#ifndef CRUISE_CONTROL_ALGORITHM_HEADER_BASIC_CRUISE_CONTROL_H
#define CRUISE_CONTROL_ALGORITHM_HEADER_BASIC_CRUISE_CONTROL_H

#include <memory>

#include "i_cruise_strategy.h"

namespace cruise_control
{

class BasicCruiseControl : public ICruiseStrategy
{
public:
    BasicCruiseControl(std::shared_ptr<CruiseConfiguration> cruise_param);
    ~BasicCruiseControl() = default;
    BasicCruiseControl(const BasicCruiseControl&) = delete;
    BasicCruiseControl& operator=(const BasicCruiseControl&) = delete;
    BasicCruiseControl(const BasicCruiseControl&&) = delete;
    BasicCruiseControl& operator=(const BasicCruiseControl&&) = delete;

    cruise_acceleration GetAcceleration(const cruise_speed speed) override;

    bool IsCruiseAlgoParamExist(std::string parameter) override;

    int32_t GetCruiseAlgoParamValue(std::string parameter) override;

private:
    std::shared_ptr<CruiseConfiguration> algo_cruise_param_;

    units::velocity::kilometers_per_hour_t set_speed_;
    units::mass::kilogram_t vehicle_mass_;
    units::force::newton_t driving_force_;
    units::force::newton_t resistance_coefficient_;


};
} //namespace cruise_control
#endif // CRUISE_CONTROL_ALGORITHM_HEADER_BASIC_CRUISE_CONTROL_H

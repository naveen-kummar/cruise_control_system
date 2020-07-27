/*
*   @file i_cruise_strategy.h
*	@brief Defines the interface for cruise control algo. This represents the 
*   top most interface in the decorator pattern which is implemented by both
*   Basic Implementer and concrete decorator
*/
#ifndef CRUISE_CONTROL_ALGORITHM_HEADER_I_CRUISE_STRATEGY_H
#define CRUISE_CONTROL_ALGORITHM_HEADER_I_CRUISE_STRATEGY_H

#include <string>
#include "units.h"

#include "../../configuration/header/cruise_configuration.h"

namespace cruise_control
{

using cruise_acceleration = units::acceleration::meters_per_second_squared_t;
using cruise_speed = units::velocity::kilometers_per_hour_t;

class ICruiseStrategy
{
public:
    ICruiseStrategy() = default;
    virtual ~ICruiseStrategy() = default;
    ICruiseStrategy(const ICruiseStrategy&) = delete;
    ICruiseStrategy& operator=(const ICruiseStrategy&) = delete;
    ICruiseStrategy(const ICruiseStrategy&&) = delete;
    ICruiseStrategy& operator=(const ICruiseStrategy&&) = delete;

    virtual cruise_acceleration GetAcceleration(const cruise_speed speed) = 0;

    virtual bool IsCruiseAlgoParamExist(std::string parameter) = 0;

    virtual int32_t GetCruiseAlgoParamValue(std::string parameter) = 0;
};
} //namespace cruise_control
#endif // CRUISE_CONTROL_ALGORITHM_HEADER_I_CRUISE_STRATEGY_H

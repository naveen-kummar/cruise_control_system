/*
*   @file cruise_algo_factory.h
*	@brief Declares factory method for generating algo for calculating acceleration
*   based on received input speed
*/
#ifndef CRUISE_CONTROL_ALGORITHM_HEADER_CRUISE_ALGO_FACTORY_H
#define CRUISE_CONTROL_ALGORITHM_HEADER_CRUISE_ALGO_FACTORY_H

#include "basic_cruise_control.h"
#include "concrete_strategy_decorator.h"
#include "jerk_control_decorator.h"
#include "../../configuration/header/cruise_configuration.h"

namespace cruise_control
{
class CruiseAlgoFactory
{
public:
    CruiseAlgoFactory() = default;
    ~CruiseAlgoFactory() = default;
    CruiseAlgoFactory(const CruiseAlgoFactory&) = delete;
    CruiseAlgoFactory& operator=(const CruiseAlgoFactory&) = delete;
    CruiseAlgoFactory(const CruiseAlgoFactory&&) = delete;
    CruiseAlgoFactory& operator=(const CruiseAlgoFactory&&) = delete;
        
    static std::unique_ptr<ICruiseStrategy> CreateCruiseAlgo(
                        std::shared_ptr<CruiseConfiguration> config_param);

};
} //namespace cruise_control
#endif //CRUISE_CONTROL_ALGORITHM_HEADER_CRUISE_ALGO_FACTORY_H

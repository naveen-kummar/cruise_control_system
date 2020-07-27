
/*
*   @file concrete_strategy_decorator.cpp
*	@brief Defines concrete decortor which will be used by other advanced algorithms for
*   cruise control
*/
#include "concrete_strategy_decorator.h"

namespace cruise_control
{

ConreteDecorator::ConreteDecorator(std::unique_ptr<ICruiseStrategy> cruise_algo)
        : basic_algo_{ std::move(cruise_algo) }
{

}

cruise_acceleration ConreteDecorator::GetAcceleration(const cruise_speed speed) 
{
    return basic_algo_->GetAcceleration(speed);
}

bool ConreteDecorator::IsCruiseAlgoParamExist(std::string parameter)
{
    return basic_algo_->IsCruiseAlgoParamExist(parameter);
}

int32_t ConreteDecorator::GetCruiseAlgoParamValue(std::string parameter)
{
    return basic_algo_->GetCruiseAlgoParamValue(parameter);
}

} //namespace cruise_control

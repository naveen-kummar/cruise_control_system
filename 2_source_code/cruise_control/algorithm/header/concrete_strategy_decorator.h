/*
*   @file concrete_strategy_decorator.h
*	@brief declare concrete decortor which will be used by other derived
*   decorators to modify the acceleration based on parameter configured
*   for the decorator
*/
#ifndef CRUISE_CONTROL_ALGORITHM_HEADER_ABSTRACT_STRATEGY_DECORATOR_H
#define CRUISE_CONTROL_ALGORITHM_HEADER_ABSTRACT_STRATEGY_DECORATOR_H
#include "i_cruise_strategy.h"
#include <memory>

namespace cruise_control
{
class ConreteDecorator : public ICruiseStrategy
{

public:
    ConreteDecorator(std::unique_ptr<ICruiseStrategy> cruise_algo);
    ~ConreteDecorator() = default;
    ConreteDecorator(const ConreteDecorator&) = delete;
    ConreteDecorator& operator=(const ConreteDecorator&) = delete;
    ConreteDecorator(const ConreteDecorator&&) = delete;
    ConreteDecorator& operator=(const ConreteDecorator&&) = delete;

    cruise_acceleration GetAcceleration(const cruise_speed speed) override;

    bool IsCruiseAlgoParamExist(std::string parameter) override;

    int32_t GetCruiseAlgoParamValue(std::string parameter) override;

protected:
    std::unique_ptr<ICruiseStrategy> basic_algo_;
};
} //namespace cruise_control
#endif //CRUISE_CONTROL_ALGORITHM_HEADER_ABSTRACT_STRATEGY_DECORATOR_H

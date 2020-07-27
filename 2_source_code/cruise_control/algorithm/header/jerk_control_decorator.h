/*
*   @file jerk_control_decorator.h
*	@brief Declares algorithm adjusting the accelerationb based on JERK Parameter
*/
#ifndef CRUISE_CONTROL_ALGORITHM_HEADER_JERK_CONTROL_DECORATOR_H
#define CRUISE_CONTROL_ALGORITHM_HEADER_JERK_CONTROL_DECORATOR_H
#include "concrete_strategy_decorator.h"

namespace cruise_control
{
class JerkControlDecorator : public ConreteDecorator
{
public:
    JerkControlDecorator(std::unique_ptr<ICruiseStrategy> decorated_algo);
    ~JerkControlDecorator() = default;
    JerkControlDecorator(const JerkControlDecorator&) = delete;
    JerkControlDecorator& operator=(const JerkControlDecorator&) = delete;
    JerkControlDecorator(const JerkControlDecorator&&) = delete;
    JerkControlDecorator& operator=(const JerkControlDecorator&&) = delete;

    cruise_acceleration GetAcceleration(const cruise_speed speed) override;

};
} //namespace cruise_control
#endif //CRUISE_CONTROL_ALGORITHM_HEADER_JERK_CONTROL_DECORATOR_H

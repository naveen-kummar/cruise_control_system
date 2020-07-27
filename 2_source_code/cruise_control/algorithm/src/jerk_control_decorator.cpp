
/*
*   @file jerk_control_decorator.cpp
*   @brief Defines algorithm for jerk control. This algorithm adjust the 
*   acceleration value calculated by basic cruise control algorithm based
*   on jerk value configured
*/
#include "jerk_control_decorator.h"

namespace cruise_control
{

constexpr float kSmoothAdjustment{2.0F};
constexpr float kNormalAdjustment{4.0F};
constexpr float kFastAdjustment{6.0F};
constexpr int32_t kSmoothAcceleration{1};
constexpr int32_t kNormalAcceleration{2};
constexpr int32_t kFastAcceleration{3};

JerkControlDecorator::JerkControlDecorator(std::unique_ptr<ICruiseStrategy> decorated_algo) :
    ConreteDecorator(std::move(decorated_algo))
{

}

cruise_acceleration JerkControlDecorator::GetAcceleration(const cruise_speed speed) 
{
     cruise_acceleration return_acceleration{basic_algo_->GetAcceleration(speed)};
     int32_t jerk_value = basic_algo_->IsCruiseAlgoParamExist(kParamJerk)?
                          basic_algo_->GetCruiseAlgoParamValue(kParamJerk):
                          kNormalAcceleration;

     /*The acceleration returned by 'Basic Cruise Control Algo' is multiplied
     by a factor based on the Jerk setting*/

     if(kSmoothAcceleration == jerk_value)
     {
          return_acceleration *= kSmoothAdjustment;
     }
     else if(kFastAcceleration == jerk_value)
     {
          return_acceleration *= kFastAdjustment;   
     }
     else
     {
          return_acceleration *= kNormalAdjustment;         
     }

     return return_acceleration;
}

} //namespace cruise_control

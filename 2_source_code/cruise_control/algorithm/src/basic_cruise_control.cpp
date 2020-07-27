
/*
*   @file basic_cruise_control.cpp
*	@brief Defines basic cruise control algorithm for calculating acceleration
*   to reach the SET_SPEED based on the current vehicle speed.
*/
#include "basic_cruise_control.h"

namespace cruise_control
{
constexpr cruise_acceleration kZeroAcceleration{0.0F};
constexpr int32_t kZeroIntValue{0};

using namespace units;
using namespace units::literals;
using namespace units::time;

BasicCruiseControl::BasicCruiseControl(std::shared_ptr<CruiseConfiguration> cruise_param) : 
algo_cruise_param_{cruise_param}
{
    //Get Set Speed value from Cruise Configuration
    double set_speed = algo_cruise_param_->IsParamExist(kParamSetSpeed) ?
                        algo_cruise_param_->GetParamValue(kParamSetSpeed) :
                        kZeroIntValue;
    set_speed_ = ((units::length::kilometer_t{set_speed}) / 
                    units::time::hour_t{1});

    //Get Vehicle Mass value from Cruise Configuration
    double vehicle_mass = algo_cruise_param_->IsParamExist(kVehicleMass) ?
                        algo_cruise_param_->GetParamValue(kVehicleMass) :
                        kZeroIntValue;
    vehicle_mass_ = units::mass::kilogram_t{vehicle_mass};

    //Get Driving Force value from Cruise Configuration
    double driving_force = algo_cruise_param_->IsParamExist(kDrivingForce) ?
                        algo_cruise_param_->GetParamValue(kDrivingForce) :
                        kZeroIntValue; 
    driving_force_ = units::force::newton_t{driving_force};

    //Get Resistance Coefficient value from Cruise Configuration
    double resistance_coefficient = algo_cruise_param_->IsParamExist(kResistanceCoefficient) ?
                        algo_cruise_param_->GetParamValue(kResistanceCoefficient) :
                        kZeroIntValue; 
    resistance_coefficient_ = units::force::newton_t{resistance_coefficient};

}

cruise_acceleration BasicCruiseControl::GetAcceleration(const cruise_speed speed) 
{
    cruise_acceleration acceleration{0};
 
     //Acceleration calculation algo
    if (speed < set_speed_)
    {
        units::force::newton_t speed_adjusted_for_resistance = 
                        (resistance_coefficient_ * (unit_cast<float>(speed)));
        auto required_incremental_driving_force = driving_force_ - speed_adjusted_for_resistance;
        acceleration = required_incremental_driving_force/vehicle_mass_; 

        if(acceleration < kZeroAcceleration)
        {
            acceleration = kZeroAcceleration;
        }
    }
    
    return acceleration;
}

bool BasicCruiseControl::IsCruiseAlgoParamExist(std::string parameter)
{
    return algo_cruise_param_->IsParamExist(parameter);
}

int32_t BasicCruiseControl::GetCruiseAlgoParamValue(std::string parameter)
{
    return algo_cruise_param_->GetParamValue(parameter);
}

} //namespace cruise_control
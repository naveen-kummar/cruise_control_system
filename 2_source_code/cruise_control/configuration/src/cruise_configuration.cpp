/*
*   @file cruise_configuration.cpp
*	@brief Define functionallities to get, update and modify the configuration parameters
*/

#include "cruise_configuration.h"

namespace cruise_control
{

void CruiseConfiguration::SetDefaultValues()
{
    AddConfigParam(kVehicleMass, kDefaultMass);
    AddConfigParam(kParamSetSpeed, kDefaultSetSpeed);
    AddConfigParam(kParamJerk, kDefaultJerk);
    AddConfigParam(kResistanceCoefficient, kDefaultResistanceCoeff);
    AddConfigParam(kDrivingForce, KDefaultDrivingForce);
}

bool CruiseConfiguration::IsParamExist(const std::string &param_name)
{
    bool return_value{false};
    auto iter = cruise_parameters_.find(param_name);

    if(iter != cruise_parameters_.end())
    {
        return_value = true;
    }

    return return_value;

}

int32_t CruiseConfiguration::GetParamValue(const std::string &param_name)
{
    if(IsParamExist(param_name) == false)
    {
        throw std::invalid_argument("Received invalid Parameter request");
    }
    
    int32_t return_value = cruise_parameters_[param_name];
    return return_value;
}

void CruiseConfiguration::AddConfigParam(const std::string &param_name, 
                                                const float param_value)
{
    cruise_parameters_.emplace(param_name,param_value);
}

void CruiseConfiguration::RemoveConfigParam(const std::string &param_name)
{
    cruise_parameters_.erase(param_name);
}

int CruiseConfiguration::GetCount()
{
    return cruise_parameters_.size();
}

cruise_param& CruiseConfiguration::GetConfigList()
{
    return cruise_parameters_;
}

void CruiseConfiguration::SetParamValue(const std::string &param_name, const float value)
{
    if(IsParamExist(param_name) == false)
    {
        throw std::invalid_argument("Received invalid Parameter request");
    }

    cruise_parameters_[param_name] = value;
}

}// namespace cruise_control

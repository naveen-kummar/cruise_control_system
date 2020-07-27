/*
*   @file cruise_configuration.h
*	@brief Define default parameters for Cruise Control and Vehicle Stub
*   and declares functionallities to get, update and modify them
*/

#ifndef CRUISE_CONTROL_CONFIGURATION_HEADER_CRUISE_CONFIGURATION_H
#define CRUISE_CONTROL_CONFIGURATION_HEADER_CRUISE_CONFIGURATION_H
#include <unordered_map>
#include <string>
#include <cstdint>

namespace cruise_control
{

constexpr char kVehicleMass[] = "VEHICLE_MASS";
constexpr char kParamSetSpeed[] = "SET_SPEED";
constexpr char kParamJerk[] = "JERK";
constexpr char kResistanceCoefficient[] = "RESISTANCE_COEFF";
constexpr char kDrivingForce[] = "DRIVING_FORCE";
constexpr int32_t kDefaultSetSpeed{80};
constexpr int32_t kDefaultJerk{2};
constexpr int32_t kDefaultMass{1000};
constexpr int32_t kDefaultResistanceCoeff{10};
constexpr int32_t KDefaultDrivingForce{1000};


using cruise_param =  std::unordered_map<std::string, int32_t>;

class CruiseConfiguration
{
public:
    CruiseConfiguration() = default;
    ~CruiseConfiguration() = default;
    CruiseConfiguration(const CruiseConfiguration&) = default;
    CruiseConfiguration& operator=(const CruiseConfiguration&) = default;
    CruiseConfiguration(CruiseConfiguration&&) = delete;
    CruiseConfiguration& operator=(CruiseConfiguration&&) = delete;

    void SetDefaultValues();
    bool IsParamExist(const std::string &param_name);
    int32_t GetParamValue(const std::string &param_name);
    void AddConfigParam(const std::string &param_name, const float param_value);
    void RemoveConfigParam(const std::string &param_name);
    int GetCount();
    cruise_param& GetConfigList();
    void SetParamValue(const std::string &param_name, const float value);


private:
    cruise_param cruise_parameters_;

};
} //namespace cruise_control
#endif //CRUISE_CONTROL_CONFIGURATION_HEADER_CRUISE_CONFIGURATION_H

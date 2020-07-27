/*
*   @file cruise_configuration_test.cpp
*	@brief Defines test functions to validate configuration manager
*/
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include "pthread.h"

#include "cruise_configuration.h"

using namespace cruise_control;

constexpr int32_t kNullValue{0};

class CruiseConfigurationTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cruise_config_ =
            std::make_unique<CruiseConfiguration>();
    }

    std::unique_ptr<CruiseConfiguration> cruise_config_;

};

TEST_F(CruiseConfigurationTest, TestDefaultValues)
{
    //ACT
    cruise_config_->SetDefaultValues();

    //ASSERT
    EXPECT_EQ(kDefaultMass , cruise_config_->GetParamValue(kVehicleMass));
    EXPECT_EQ(kDefaultJerk , cruise_config_->GetParamValue(kParamJerk));
    EXPECT_EQ(kDefaultResistanceCoeff , cruise_config_->GetParamValue(kResistanceCoefficient));
    EXPECT_EQ(KDefaultDrivingForce , cruise_config_->GetParamValue(kDrivingForce));
    EXPECT_EQ(kDefaultSetSpeed , cruise_config_->GetParamValue(kParamSetSpeed));    

}

TEST_F(CruiseConfigurationTest, TestNonExistentValue)
{
    //ARRANGE
    
    //ACT
    try
    {
        cruise_config_->GetParamValue("Non Existent Parameter");
    }
    catch(std::invalid_argument const &err)
    {
        //ASSERT
        EXPECT_EQ(std::string{err.what()}, std::string{"Received invalid Parameter request"});
    }
    catch(...)
    {
        FAIL() << "Unknow exception while getting parameter value";
    } 

}

TEST_F(CruiseConfigurationTest, TestErase)
{
    //ARRANGE
    cruise_config_->SetDefaultValues();

    //ASSERT
    EXPECT_EQ(kDefaultMass , cruise_config_->GetParamValue(kVehicleMass));
    
    //ACT
    cruise_config_->RemoveConfigParam(kVehicleMass);
    
    //ACT
    try
    {
        cruise_config_->GetParamValue(kVehicleMass);
    }
    catch(std::invalid_argument const &err)
    {
        //ASSERT
        EXPECT_EQ(std::string{err.what()}, std::string{"Received invalid Parameter request"});
    }
    catch(...)
    {
        FAIL() << "Unknow exception while getting parameter value";
    }  

}

TEST_F(CruiseConfigurationTest, TestSetValue)
{
    //ARRANGE
    cruise_config_->SetDefaultValues();
    int32_t test_value{77};

    //ASSERT
    EXPECT_EQ(kDefaultMass , cruise_config_->GetParamValue(kVehicleMass));
    
    //ACT
    cruise_config_->SetParamValue(kVehicleMass, test_value);
    
    //ASSERT
    EXPECT_EQ(test_value , cruise_config_->GetParamValue(kVehicleMass));   

}



int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

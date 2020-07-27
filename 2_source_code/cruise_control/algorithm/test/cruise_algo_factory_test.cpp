/*
*   @file cruise_algo_factory_test.cpp
*	@brief Defines test functions to validate cruise control algorithm
*/
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "cruise_configuration.h"
#include "cruise_algo_factory.h"
#include "units.h"

#include <iostream>

using namespace units;
using namespace cruise_control;

constexpr float kZeroAcceleration{0.0F};
constexpr float kSmoothAcceleration{1.0F};
constexpr float kNormalAcceleration{2.0F};
constexpr float kFastAcceleration{3.0F}; 

class CruiseAlgoTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        config_manager_ =
            std::make_unique<CruiseConfiguration>();
    }

    std::unique_ptr<CruiseConfiguration> config_manager_;

};

TEST_F(CruiseAlgoTest, BasicAlgoNoSetSpeed)
{
    //ARRANGE
    cruise_speed vehicle_speed{0.0F};
    int32_t set_speed{0};

    config_manager_->SetDefaultValues();
    config_manager_->SetParamValue(kParamSetSpeed, set_speed);
    

    std::unique_ptr<ICruiseStrategy> cruise_algo
        = std::move(CruiseAlgoFactory::CreateCruiseAlgo(std::move(config_manager_)));

    //ACT
    cruise_acceleration acceleration = cruise_algo->GetAcceleration(vehicle_speed);

    //ASSERT
    EXPECT_FLOAT_EQ(kZeroAcceleration, unit_cast<float>(acceleration));

}


TEST_F(CruiseAlgoTest, BasicAlgoDefaultSetSpeed)
{
    //ARRANGE
    cruise_speed vehicle_speed{0.0F};

    config_manager_->SetDefaultValues();

    std::unique_ptr<ICruiseStrategy> cruise_algo
        = std::move(CruiseAlgoFactory::CreateCruiseAlgo(std::move(config_manager_)));

    //ACT
    cruise_acceleration acceleration = cruise_algo->GetAcceleration(vehicle_speed);

    //ASSERT
    EXPECT_FLOAT_EQ(4.0F, unit_cast<float>(acceleration));

}

TEST_F(CruiseAlgoTest, BasicAlgoSmoothPickup)
{
    //ARRANGE
    cruise_speed vehicle_speed{10.0F};
    config_manager_->SetDefaultValues();

    float pickup_mode = kSmoothAcceleration;
    config_manager_->SetParamValue(kParamJerk, pickup_mode);

    std::unique_ptr<ICruiseStrategy> cruise_algo
        = std::move(CruiseAlgoFactory::CreateCruiseAlgo(std::move(config_manager_)));

    //ACT
    cruise_acceleration acceleration = cruise_algo->GetAcceleration(vehicle_speed);

    //ASSERT
    EXPECT_FLOAT_EQ(1.8F, unit_cast<float>(acceleration));

}


TEST_F(CruiseAlgoTest, BasicAlgoNormalPickup)
{
    //ARRANGE
    cruise_speed vehicle_speed{10.0F};
    config_manager_->SetDefaultValues();

    float pickup_mode = kNormalAcceleration;
    config_manager_->SetParamValue(kParamJerk, pickup_mode);

    std::unique_ptr<ICruiseStrategy> cruise_algo
        = std::move(CruiseAlgoFactory::CreateCruiseAlgo(std::move(config_manager_)));

    //ACT
    cruise_acceleration acceleration = cruise_algo->GetAcceleration(vehicle_speed);

    //ASSERT
    EXPECT_FLOAT_EQ(3.6F, unit_cast<float>(acceleration));

}

TEST_F(CruiseAlgoTest, BasicAlgoSuperPickup)
{
    //ARRANGE
    cruise_speed vehicle_speed{10.0F};
    config_manager_->SetDefaultValues();

    float pickup_mode = kFastAcceleration;
    config_manager_->SetParamValue(kParamJerk, pickup_mode);

    std::unique_ptr<ICruiseStrategy> cruise_algo
        = std::move(CruiseAlgoFactory::CreateCruiseAlgo(std::move(config_manager_)));

    //ACT
    cruise_acceleration acceleration = cruise_algo->GetAcceleration(vehicle_speed);

    //ASSERT
    EXPECT_FLOAT_EQ(5.4F, unit_cast<float>(acceleration));

}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

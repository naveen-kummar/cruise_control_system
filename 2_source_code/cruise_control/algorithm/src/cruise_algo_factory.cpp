
/*
*	@file cruise_algo_factory.cpp
*	@brief Defines factory method to create appropriate cruise algorithm
*   object based on the configured cruise parameters
*/
#include "cruise_algo_factory.h"

namespace cruise_control
{

std::unique_ptr<ICruiseStrategy> CruiseAlgoFactory::CreateCruiseAlgo(
                                            std::shared_ptr<CruiseConfiguration> config_param)
{
    std::unique_ptr<ICruiseStrategy> cruise_algo_output = 
                    std::make_unique<BasicCruiseControl>(config_param);

    if (config_param->IsParamExist(kParamJerk) == true)
    {
        cruise_algo_output =
            std::make_unique<JerkControlDecorator>(std::move(cruise_algo_output));

    }

    return cruise_algo_output;
}

} //namespace cruise_control

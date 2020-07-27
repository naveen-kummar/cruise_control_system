/*
*   @file main_node.cpp
*	@brief This is entry file for vehicle stub implementing 'main' function.
*/
#include "../header/vehicle_stub_runner.h"

using namespace cruise_control;

int main(int argc, char* argv[])
{
    try
    {
        VehicleStubRunner::Run(argc, argv);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Vehicle Stub main node execution error = "<< e.what() << '\n';
    }

    return 0;
}

/*
*   @file
*	@brief This is entry file for cruise controller implementing 'main' function.
*/

#include "../header/cruise_node_runner.h"

using namespace cruise_control;

int main(int argc, char* argv[])
{
    try
    {
        CruiseNodeRunner::Run(argc, argv);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Cruise control main node execution error = "<< e.what() << '\n';
    }

    return 0;
}

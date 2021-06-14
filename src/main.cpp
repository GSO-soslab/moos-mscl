#include "MOOS/libMOOS/App/MOOSApp.h"
#include "MicrostrainMoos.h"

int main(int argc, char *argv[]) {
    
    MOOS::CommandLineParser P(argc, argv);
    
    std::string mission_file = P.GetFreeParameter(0, "Mission.moos");
    
    std::string app_name = P.GetFreeParameter(1, "iMicrostrainIMU");
    
    auto imu_node = std::make_shared<MicrostrainMoos>();

    imu_node->Run(app_name.c_str(), mission_file.c_str());

    return EXIT_SUCCESS;
}
#ifndef MICROSTRAIN_MOOS_H
#define MICROSTRAIN_MOOS_H

#include "string"
#include "thread"
#include "memory"
#include "iostream"

#include "MOOS/libMOOS/App/MOOSApp.h"

#include "Microstrain.h"

#define MOOS_KEY_START_CALIBRATION "MICROSTRAIN_CALIBRATION"


class MicrostrainMoos :
        public std::enable_shared_from_this<MicrostrainMoos>,
        public CMOOSApp
{

public:
    MicrostrainMoos();
    ~MicrostrainMoos() override = default;

    void publish_filtered(filter_data_t d);

    bool publish_imu(imu_data_t d);

protected:
    
    bool OnStartUp() override;
    
    bool OnNewMail(MOOSMSG_LIST & Mail) override;

    bool OnConnectToServer() override;

    bool Iterate() override;


private:


    Microstrain m_imu;

    std::string m_imu_port;

    int m_imu_baudrate;

    std::string m_param;

    MOOS::MOOSAsyncCommClient m_comms;

    std::thread m_imu_thread;
};

#endif //MICROSTRAIN_MOOS_H

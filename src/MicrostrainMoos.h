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

    void publish_imu(imu_data_t d);

protected:
    
    bool OnStartUp() override;
    
    bool OnNewMail(MOOSMSG_LIST & Mail) override;

    bool OnConnectToServer() override;

    bool Iterate() override;


private:


    Microstrain m_imu;

    std::string m_imu_port;

    std::string m_prefix;
   
    bool m_publish_raw;
    
    bool m_publish_filter;
    
    int m_imu_baudrate;
    
    float m_publish_frequency;
    
    float m_imu_frequency;
    
    float m_filter_frequency;
    
    std::string m_param;

    MOOS::MOOSAsyncCommClient m_comms;

    std::thread m_imu_thread;

    std::string m_name_x_accel;
    std::string m_name_y_accel;
    std::string m_name_z_accel;


    std::string m_name_x_gyro;
    std::string m_name_y_gyro;
    std::string m_name_z_gyro;

    std::string m_name_x_mag;
    std::string m_name_y_mag;
    std::string m_name_z_mag;

    std::string m_name_w_quat;
    std::string m_name_x_quat;
    std::string m_name_y_quat;
    std::string m_name_z_quat;

    std::string m_name_roll;
    std::string m_name_pitch;
    std::string m_name_yaw;



};

#endif //MICROSTRAIN_MOOS_H

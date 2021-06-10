#include "MicrostrainMoos.h"
#include "thread"
#include "chrono"

MicrostrainMoos::MicrostrainMoos() = default;

bool MicrostrainMoos::OnStartUp() {



    if(!m_MissionReader.GetConfigurationParam("baudrate", m_imu_baudrate)){
        m_imu_baudrate = 115200;
    }
    if(!m_MissionReader.GetConfigurationParam("port", m_imu_port)){
        m_imu_port = "/dev/ttyACM0";
    }
    m_imu = Microstrain(m_imu_port, m_imu_baudrate, shared_from_this());

    m_imu.initialize();

    m_imu.configure();

    m_imu_thread = std::thread(&Microstrain::run, m_imu);

    return true;
}

bool MicrostrainMoos::OnNewMail(MOOSMSG_LIST &Mail) {

    for(const auto q : Mail) {
        if(q.m_sKey == MOOS_KEY_START_CALIBRATION) {
            m_imu.calibrate();
        }
    }


    return true;
}

bool MicrostrainMoos::OnConnectToServer() {
    bool r = true;
    // r &= Register("*","*", 0);
    return r;
}

bool MicrostrainMoos::Iterate() {
    Notify("X", std::chrono::system_clock::now().time_since_epoch().count());
    return true;
}

void MicrostrainMoos::publish_filtered(filter_data_t d) {
    Notify("MICROSTRAIN_ROLL", d.roll);
    Notify("MICROSTRAIN_PITCH", d.pitch);
    Notify("MICROSTRAIN_YAW", d.yaw);
}

bool MicrostrainMoos::publish_imu(imu_data_t d) {
    Notify("MICROSTRAIN_LINEAR_ACCEL_X", d.linear_accel_x);
    Notify("MICROSTRAIN_LINEAR_ACCEL_Y", d.linear_accel_y);
    Notify("MICROSTRAIN_LINEAR_ACCEL_Z", d.linear_accel_z);


    Notify("MICROSTRAIN_ANGULAR_X", d.angular_vel_x);
    Notify("MICROSTRAIN_ANGULAR_Y", d.angular_vel_y);
    Notify("MICROSTRAIN_ANGULAR_Z", d.angular_vel_z);

    Notify("MICROSTRAIN_MAG_X", d.mag_x);
    Notify("MICROSTRAIN_MAG_Y", d.mag_y);
    Notify("MICROSTRAIN_MAG_Z", d.mag_z);

    Notify("MICROSTRAIN_QUAT_W", d.quat_w);
    Notify("MICROSTRAIN_QUAT_X", d.quat_x);
    Notify("MICROSTRAIN_QUAT_Y", d.quat_y);
    Notify("MICROSTRAIN_QUAT_Z", d.quat_z);
}
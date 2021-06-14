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
    if(!m_MissionReader.GetConfigurationParam("prefix", m_prefix)) {
        m_prefix = "MS";
    }
    if(! m_MissionReader.GetConfigurationParam("publish_raw", m_publish_raw)) {
        m_publish_raw = true;
    }
    if(! m_MissionReader.GetConfigurationParam("publish_filter", m_publish_filter)) {
        m_publish_filter = true;
    }
    if(! m_MissionReader.GetConfigurationParam("publish_frequency", m_publish_frequency)) {
        m_publish_frequency = 50;
    }
    if(! m_MissionReader.GetConfigurationParam("imu_frequency", m_imu_frequency)) {
        m_imu_frequency = 50;
    }
    if(! m_MissionReader.GetConfigurationParam("filter_frequency", m_filter_frequency)) {
        m_filter_frequency = 50;
    }
    m_imu = Microstrain(m_imu_port, m_imu_baudrate, shared_from_this());
    m_imu.set_rate(m_publish_frequency);
    m_imu.set_imu_rate(m_imu_frequency);
    m_imu.set_filter_rate(m_filter_frequency);
    
    m_imu.initialize();

    m_imu.configure();

    m_imu_thread = std::thread(&Microstrain::run, m_imu);

    m_name_roll = m_prefix + "_ROLL";
    m_name_pitch = m_prefix + "_PITCH";
    m_name_yaw = m_prefix + "_YAW";

    m_name_x_accel = m_prefix + "_X_ACCEL";
    m_name_y_accel = m_prefix + "_Y_ACCEL";
    m_name_z_accel = m_prefix + "_Z_ACCEL";

    m_name_x_gyro = m_prefix + "_X_GYRO";
    m_name_y_gyro = m_prefix + "_Y_GYRO";
    m_name_z_gyro = m_prefix + "_Z_GYRO";

    m_name_z_quat = m_prefix + "_W_QUAT";
    m_name_x_quat = m_prefix + "_X_QUAT";
    m_name_y_quat = m_prefix + "_Y_QUAT";
    m_name_z_quat = m_prefix + "_Z_QUAT";

    m_name_x_quat = m_prefix + "_X_MAG";
    m_name_y_quat = m_prefix + "_Y_MAG";
    m_name_z_quat = m_prefix + "_Z_MAG";

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
    r &= Register(MOOS_KEY_START_CALIBRATION, 0);
    return r;
}

bool MicrostrainMoos::Iterate() {
    return true;
}

void MicrostrainMoos::publish_filtered(filter_data_t d) {
    if(m_publish_filter) {
        Notify(m_name_roll, d.roll);
        Notify(m_name_pitch, d.pitch);
        Notify(m_name_yaw, d.yaw);
    }
}

void MicrostrainMoos::publish_imu(imu_data_t d) {
    if(m_publish_raw) {
        Notify(m_name_x_accel, d.linear_accel_x);
        Notify(m_name_y_accel, d.linear_accel_y);
        Notify(m_name_z_accel, d.linear_accel_z);

        Notify(m_name_x_gyro, d.angular_vel_x);
        Notify(m_name_y_gyro, d.angular_vel_y);
        Notify(m_name_z_gyro, d.angular_vel_z);
    
        Notify(m_name_x_mag, d.mag_x);
        Notify(m_name_y_mag, d.mag_y);
        Notify(m_name_z_mag, d.mag_z);
    
        Notify(m_name_w_quat, d.quat_w);
        Notify(m_name_x_quat, d.quat_x);
        Notify(m_name_y_quat, d.quat_y);
        Notify(m_name_z_quat, d.quat_z);
    }
}
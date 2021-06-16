#include "MicrostrainMoos.h"
#include "thread"
#include "chrono"

MicrostrainMoos::MicrostrainMoos() = default;

bool MicrostrainMoos::OnStartUp() {



    if(!m_MissionReader.GetConfigurationParam("AppTick", m_app_tick))
    {
        MOOSTrace("Warning, AppTick not set.\n");
    }

    if(!m_MissionReader.GetConfigurationParam("CommsTick", m_comms_tick))
    {
        MOOSTrace("Warning, CommsTick not set.\n");
    }

    SetAppFreq(m_app_tick);
    SetCommsFreq(m_comms_tick);


    std::string conn_type_param;
    ConnectionType conn_type;
    if(!m_MissionReader.GetConfigurationParam("connection_type", conn_type_param)){
        conn_type_param = "SERIAL";
    }
    if(conn_type_param == "SERIAL") {
        conn_type = ConnectionType::SERIAL;
    } else if (conn_type_param == "TCP") {
        conn_type = ConnectionType::TCP;
    } else {
        conn_type = ConnectionType::SERIAL;
    }
    if(!m_MissionReader.GetConfigurationParam("tcp_addr", m_tcp_addr)){
        m_tcp_addr = "localhost";
    }
    if(!m_MissionReader.GetConfigurationParam("tcp_port", m_tcp_port)){
        m_tcp_port = 4161;
    }
    if(!m_MissionReader.GetConfigurationParam("baudrate", m_imu_baudrate)){
        m_imu_baudrate = 115200;
    }
    if(!m_MissionReader.GetConfigurationParam("port", m_imu_port)){
        m_imu_port = "/dev/microstrain";
    }
    if(!m_MissionReader.GetConfigurationParam("filter_prefix", m_filter_prefix)) {
        m_filter_prefix = "MS";
    }
    if(!m_MissionReader.GetConfigurationParam("raw_prefix", m_raw_prefix)) {
        m_raw_prefix = "MSRAW";
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
        m_imu_frequency = 100;
    }
    if(! m_MissionReader.GetConfigurationParam("filter_frequency", m_filter_frequency)) {
        m_filter_frequency = 50;
    }

    m_imu = Microstrain();
    m_imu.set_moos_node(shared_from_this());

    m_imu.set_connection_type(conn_type);
    m_imu.set_baudrate(m_imu_baudrate);
    m_imu.set_port(m_imu_port);
    m_imu.set_tcp_addr(m_tcp_addr);
    m_imu.set_tcp_port(m_tcp_port);

    m_imu.set_rate(m_publish_frequency);
    m_imu.set_imu_rate(m_imu_frequency);
    m_imu.set_filter_rate(m_filter_frequency);

    m_imu.initialize();

    m_imu.configure();

    m_name_roll = m_raw_prefix + "_ROLL";
    m_name_pitch =  m_raw_prefix+ "_PITCH";
    m_name_yaw = m_raw_prefix + "_YAW";

    m_name_x_accel = m_raw_prefix + "_X_ACCEL";
    m_name_y_accel = m_raw_prefix + "_Y_ACCEL";
    m_name_z_accel = m_raw_prefix + "_Z_ACCEL";

    m_name_x_gyro = m_raw_prefix + "_X_GYRO";
    m_name_y_gyro = m_raw_prefix + "_Y_GYRO";
    m_name_z_gyro = m_raw_prefix + "_Z_GYRO";

    m_name_z_quat = m_raw_prefix + "_W_QUAT";
    m_name_x_quat = m_raw_prefix + "_X_QUAT";
    m_name_y_quat = m_raw_prefix + "_Y_QUAT";
    m_name_z_quat = m_raw_prefix + "_Z_QUAT";

    m_name_x_quat = m_raw_prefix + "_X_MAG";
    m_name_y_quat = m_raw_prefix + "_Y_MAG";
    m_name_z_quat = m_raw_prefix + "_Z_MAG";

    m_name_f_roll = m_filter_prefix + "_ROLL";
    m_name_f_pitch = m_filter_prefix + "_PITCH";
    m_name_f_yaw = m_filter_prefix + "_YAW";

    m_name_f_x_accel = m_filter_prefix + "_X_ACCEL";
    m_name_f_y_accel = m_filter_prefix + "_Y_ACCEL";
    m_name_f_z_accel = m_filter_prefix + "_Z_ACCEL";

    m_name_f_x_gyro = m_filter_prefix + "_X_GYRO";
    m_name_f_y_gyro = m_filter_prefix + "_Y_GYRO";
    m_name_f_z_gyro = m_filter_prefix + "_Z_GYRO";

    m_name_f_w_quat = m_filter_prefix + "_W_QUAT";
    m_name_f_x_quat = m_filter_prefix + "_X_QUAT";
    m_name_f_y_quat = m_filter_prefix + "_Y_QUAT";
    m_name_f_z_quat = m_filter_prefix + "_Z_QUAT";

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
    r &= Register(MOOS_KEY_START_CALIBRATION, 0);
    return r;
}

bool MicrostrainMoos::Iterate() {
    return true;
}

void MicrostrainMoos::publish_filtered(imu_data_t &d) {
    if(m_publish_filter) {
        Notify(m_name_f_roll, d.roll, d.time);
        Notify(m_name_f_pitch, d.pitch, d.time);
        Notify(m_name_f_yaw, d.yaw, d.time);

        Notify(m_name_f_x_accel, d.linear_accel_x, d.time);
        Notify(m_name_f_y_accel, d.linear_accel_y, d.time);
        Notify(m_name_f_z_accel, d.linear_accel_z, d.time);

        Notify(m_name_f_x_gyro, d.angular_vel_x, d.time);
        Notify(m_name_f_y_gyro, d.angular_vel_y, d.time);
        Notify(m_name_f_z_gyro, d.angular_vel_z, d.time);

        Notify(m_name_f_w_quat, d.quat_w, d.time);
        Notify(m_name_f_x_quat, d.quat_x, d.time);
        Notify(m_name_f_y_quat, d.quat_y, d.time);
        Notify(m_name_f_z_quat, d.quat_z, d.time);
    }
}

void MicrostrainMoos::publish_imu(imu_data_t &d) {
    if(m_publish_raw) {
        Notify(m_name_roll, d.roll, d.time);
        Notify(m_name_pitch, d.pitch, d.time);
        Notify(m_name_yaw, d.yaw, d.time);


        Notify(m_name_x_accel, d.linear_accel_x, d.time);
        Notify(m_name_y_accel, d.linear_accel_y, d.time);
        Notify(m_name_z_accel, d.linear_accel_z, d.time);

        Notify(m_name_x_gyro, d.angular_vel_x, d.time);
        Notify(m_name_y_gyro, d.angular_vel_y, d.time);
        Notify(m_name_z_gyro, d.angular_vel_z, d.time);
    
        Notify(m_name_x_mag, d.mag_x, d.time);
        Notify(m_name_y_mag, d.mag_y, d.time);
        Notify(m_name_z_mag, d.mag_z, d.time);
    
        Notify(m_name_w_quat, d.quat_w, d.time);
        Notify(m_name_x_quat, d.quat_x, d.time);
        Notify(m_name_y_quat, d.quat_y, d.time);
        Notify(m_name_z_quat, d.quat_z, d.time);
    }
}
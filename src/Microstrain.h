#ifndef MICROSTRAIN_H
#define MICROSTRAIN_H

#include "mscl/mscl.h"
#include "cstdio"
#include "unistd.h"
#include "ctime"
#include "iostream"
#include "fstream"
#include "MOOS/libMOOS/App/MOOSApp.h"

#define DECLINATION_SOURCE_NONE 1
#define DECLINATION_SOURCE_MAGNETIC 2
#define DECLINATION_SOURCE_MANUAL 3

#define HEADING_SOURCE_NONE 0
#define HEADING_SOURCE_MAGNETIC 1
#define HEADING_SOURCE_GNSS 2

#define DYNAMICS_MODE_PORTABLE 1 //default
#define DYNAMICS_MODE_AUTOMOTIVE 2
#define DYNAMICS_MODE_AIRBORNE 3 // < 2Gs
#define DYNAMICS_MODE_AIRBORNE_HIGH 4 // < 4Gs

#define FILTER_ADAPTIVE_LEVEL_OFF 0
#define FILTER_ADAPTIVE_LEVEL_CONSERVATIVE 1
#define FILTER_ADAPTIVE_LEVEL_MODERATE 2
#define FILTER_ADAPTIVE_LEVEL_AGGRESSIVE 3

#define USTRAIN_G 9.80665

typedef struct imu_data_t {
    uint64_t time;
    float roll;
    float pitch;
    float yaw;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    float angular_vel_x;
    float angular_vel_y;
    float angular_vel_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;


    friend std::ostream& operator<<(std::ostream& os, const imu_data_t& obj)
    {
        // write obj to stream
        os << "LX: " << obj.linear_accel_x
                << " LY: " << obj.linear_accel_y
                << " LZ: " << obj.linear_accel_z
                << " AX: " << obj.angular_vel_x
                << " AY: " << obj.angular_vel_y
                << " AZ: " << obj.angular_vel_z
                << " MX: " << obj.mag_x
                << " MY: " << obj.mag_y
                << " MZ: " << obj.mag_z
                << " QW: " << obj.quat_w
                << " QX: " << obj.quat_x
                << " QY: " << obj.quat_y
                << " QZ: " << obj.quat_z;
        return os;
    }
} imu_data_t;



class MicrostrainMoos;

class Microstrain {
    public:
        
        Microstrain() = default;
        ~Microstrain() = default;

        Microstrain(std::string port, int baud);

        Microstrain(std::string port, int baud, std::shared_ptr<MicrostrainMoos> moos_node);

        void set_baudrate(int b) { m_baudrate = b;}
        int get_baudrate() { return m_baudrate; }
        
        void set_port(std::string p) { m_port = p; }
        std::string get_port() { return m_port; }
        
        void set_rate(float r) { m_rate = r; }
        float get_rate() { return m_rate; }
    
        void set_imu_rate(float r) { m_imu_data_rate = r; }
        float get_imu_rate() { return m_imu_data_rate; }
    
        void set_filter_rate(float r) { m_filter_data_rate = r; }
        float get_filter_rate() { return m_filter_data_rate; }

        void initialize();
        
        void configure();
        
        bool test_connection();
        
        void run();

        void run_once();

        void stop();

        void calibrate();

        imu_data_t get_imu_data();

        imu_data_t & get_filter_data();

    private:
    
        std::string m_port;
        int m_baudrate;
        bool m_stop = false;
        float m_rate = 100; // hz

        imu_data_t m_imu_data;

        imu_data_t m_filter_data;

        float m_initial_heading;
        float m_imu_data_rate = 100;
        int m_filter_data_rate = 100;
        float m_heading_source = HEADING_SOURCE_MAGNETIC;



        std::shared_ptr<MicrostrainMoos> m_moos_node;
        std::shared_ptr<mscl::Connection> m_connection;
        std::shared_ptr<mscl::InertialNode> m_dev;
    
        void parse_mip_packet(const mscl::MipDataPacket& packet);
        void parse_imu_packet(const mscl::MipDataPacket& packet);
        void parse_filter_packet(const mscl::MipDataPacket& packet);


};

#endif //MICROSTRAIN_H

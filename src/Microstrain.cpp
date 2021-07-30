/*
 *
 *  This source code specifically written for 3DM-GX3-25 model
 *
 */

#include "string"
#include "algorithm"
#include "vector"
#include "cmath"
#include "ctime"
#include "cstdlib"
#include "cstdint"
#include "mscl/mscl.h"
#include "chrono"
#include "thread"
#include "iostream"

#include "Microstrain.h"
#include "MicrostrainMoos.h"


Microstrain::Microstrain(std::string port, int baud) {
    m_baudrate = baud;
    m_port = port;
}

Microstrain::Microstrain(std::string port, int baud, std::shared_ptr<MicrostrainMoos> moos_node){
    m_baudrate = baud;
    m_port = port;
    m_moos_node = moos_node;
}

void Microstrain::initialize() {

    if(m_connection_type == ConnectionType::TCP) {
        m_connection = std::make_shared<mscl::Connection>
                (mscl::Connection::TcpIp(m_tcp_addr, m_tcp_port));
    }
    if(m_connection_type == ConnectionType::SERIAL) {
        m_connection = std::make_shared<mscl::Connection>
                (mscl::Connection::Serial(realpath(m_port.c_str(), 0), m_baudrate));
    }
    m_dev = std::make_shared<mscl::InertialNode>
            (*m_connection);
}

void Microstrain::configure() {

    m_dev->setToIdle();

    bool supports_filter = m_dev->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
    bool supports_imu    = m_dev->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);

    if(supports_imu && m_moos_node->get_publish_raw()) {
        mscl::SampleRate imu_rate = mscl::SampleRate::Hertz(m_imu_data_rate);
    
        std::cout << "Setting IMU data to stream at " <<  m_imu_data_rate << " hz" << std::endl;

        mscl::MipTypes::MipChannelFields ahrsChannels{
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_EULER_ANGLES,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_MAG_VEC,
                // mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_RAW_ACCEL_VEC,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_RAW_GYRO_VEC,
                mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_RAW_MAG_VEC
        };
    
        mscl::MipChannels supportedChannels;
        for(mscl::MipTypes::ChannelField channel : m_dev->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU))
        {
            if(std::find(ahrsChannels.begin(), ahrsChannels.end(), channel) != ahrsChannels.end())
            {
                supportedChannels.push_back(mscl::MipChannel(channel, imu_rate));
            }
        }

        m_dev->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, supportedChannels);

        if(m_dev->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_DECLINATION_SRC))
        {
            std::cout << "Setting Declination Source" << std::endl;
            m_dev->setDeclinationSource(mscl::GeographicSourceOptions(static_cast<mscl::InertialTypes::GeographicSourceOption>((uint8_t)DECLINATION_SOURCE_MANUAL), DECLINATION_SOURCE_MANUAL));
        }
        else
        {
            std::cout << "Note: Device does not support the declination source command." << std::endl;
        }

        m_dev->enableDataStream(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);
    }
    
    
    if(supports_filter && m_moos_node->get_publish_filter())
    {
        mscl::SampleRate filter_rate = mscl::SampleRate::Hertz(m_filter_data_rate);
        
        fprintf(stdout, "Setting Filter data to stream at %d hz\n", m_filter_data_rate);
        
        mscl::MipTypes::MipChannelFields filterChannels{
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_FILTER_STATUS,
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS,
                mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
                mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_QUAT,
                mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
                mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
                mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL,
                mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE,
                // mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_NED_RELATIVE_POS
        };
        
       
        mscl::MipChannels supportedChannels;
        for(mscl::MipTypes::ChannelField channel : m_dev->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER))
        {
            if(std::find(filterChannels.begin(), filterChannels.end(), channel) != filterChannels.end())
            {
                supportedChannels.push_back(mscl::MipChannel(channel, filter_rate));
            }
        }
        m_dev->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER, supportedChannels);

        //set dynamics mode
        if(m_dev->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
        {
            mscl::VehicleModeTypes modes = m_dev->features().supportedVehicleModeTypes();
            if (std::find(modes.begin(), modes.end(), static_cast<mscl::InertialTypes::VehicleModeType>(DYNAMICS_MODE_PORTABLE)) != modes.end())
            {
                fprintf(stdout,"Setting dynamics mode to %#04X\n", static_cast<mscl::InertialTypes::VehicleModeType>(DYNAMICS_MODE_PORTABLE));
                m_dev->setVehicleDynamicsMode(static_cast<mscl::InertialTypes::VehicleModeType>(DYNAMICS_MODE_PORTABLE));
            }
        }
        else
        {
            fprintf(stdout,"Note: The device does not support the vehicle dynamics mode command.\n");
        }
        
        //Set heading Source
        if(m_dev->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
        {
            for(mscl::HeadingUpdateOptions headingSources : m_dev->features().supportedHeadingUpdateOptions())
            {
                if(headingSources.AsOptionId() == static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(HEADING_SOURCE_MAGNETIC))
                {
                    fprintf(stdout,"Setting heading source to %#04X\n", HEADING_SOURCE_MAGNETIC);
                    m_dev->setHeadingUpdateControl(mscl::HeadingUpdateOptions(static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(HEADING_SOURCE_MAGNETIC)));
                    break;
                }
            }
            
            //Set the initial heading
            if((m_heading_source == 0) && (m_dev->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING)))
            {
                fprintf(stdout,"Setting initial heading to %f\n", m_initial_heading);
                m_dev->setInitialHeading(m_initial_heading);
            }
        }
        else
        {
            fprintf(stdout, "Note: The device does not support the heading source command.\n");
        }
        
        
        //Set the filter auto initialization, if suppored
        if(m_dev->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AUTO_INIT_CTRL))
        {
            // TODO: this part is hard coded. It might needed to be changed.
            fprintf(stdout, "Setting auto initialization to %d\n", true);
            m_dev->setAutoInitialization(true);
        }
        else
        {
            fprintf(stdout, "Note: The device does not support the filter auto initialization command.\n");
        }
       
        //Enable the filter datastream
        m_dev->enableDataStream(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
    }
   
    // todo: set rotations https://github.com/LORD-MicroStrain/ROS-MSCL/blob/3d7372e8b2341c6b9cb8c898bd125e5afd491195/ros_mscl/src/microstrain_3dm.cpp#L831-L912

    // m_dev->setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);

    m_dev->resume();
}

void Microstrain::parse_mip_packet(const mscl::MipDataPacket &packet) {
    switch (packet.descriptorSet())
    {
        case mscl::MipTypes::DataClass::CLASS_AHRS_IMU:
            if(m_moos_node->get_publish_raw()) {
                parse_imu_packet(packet);
            }
            break;
        
        case mscl::MipTypes::DataClass::CLASS_ESTFILTER:
            if(m_moos_node->get_publish_filter()) {
                parse_filter_packet(packet);
            }
            break;
        default:
            break;
    }
}


void Microstrain::parse_filter_packet(const mscl::MipDataPacket &packet) {

    //Handle time
    m_filter_data.time = double(packet.collectedTimestamp().nanoseconds()) ;

    //Get the list of data elements
    const mscl::MipDataPoints &points = packet.data();

    //Loop over data elements and map them
    for(mscl::MipDataPoint point : points)
    {
        switch(point.field())
        {
            /*
            case mscl::MipTypes::CH_FIELD_ESTFILTER_FILTER_STATUS:
            {
                if(point.qualifier() == mscl::MipTypes::CH_FILTER_STATE)
                {
                    // m_filter_status_msg.filter_state = point.as_uint16();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_DYNAMICS_MODE)
                {
                    // m_filter_status_msg.dynamics_mode = point.as_uint16();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
                {
                    // m_filter_status_msg.status_flags = point.as_uint16();
                }
            }break;
            */

            case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER:
            {
                if(point.qualifier() == mscl::MipTypes::CH_ROLL)
                {
                    m_filter_data.roll = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
                {
                    m_filter_data.pitch = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_YAW)
                {
                    m_filter_data.yaw = point.as_float();

                    // m_filter_heading_msg.heading_deg = m_curr_filter_yaw*180.0/3.14;
                    // m_filter_heading_msg.heading_rad = m_curr_filter_yaw;
                }
                else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
                {
                    // m_filter_heading_msg.status_flags = point.as_uint16();
                }
            }break;

            case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION:
            {
                mscl::Vector quaternion  = point.as_Vector();
                m_filter_data.quat_w = quaternion.as_floatAt(0);
                m_filter_data.quat_x = quaternion.as_floatAt(1);
                m_filter_data.quat_y = quaternion.as_floatAt(2);
                m_filter_data.quat_z = quaternion.as_floatAt(3);


            }break;

            case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE:
            {
                if(point.qualifier() == mscl::MipTypes::CH_X)
                {
                    m_filter_data.angular_vel_x = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Y)
                {
                    m_filter_data.angular_vel_y = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Z)
                {
                    m_filter_data.angular_vel_z = point.as_float();
                }
            }break;

            case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL:
            {
                if(point.qualifier() == mscl::MipTypes::CH_X)
                {
                    m_filter_data.linear_accel_x = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Y)
                {
                    m_filter_data.linear_accel_y = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Z)
                {
                    m_filter_data.linear_accel_z = point.as_float();
                }
            }break;

            case mscl::MipTypes::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL:
            {
                if (point.qualifier() == mscl::MipTypes::CH_X)
                {
                    m_filter_data.linear_accel_x = point.as_float();
                }
                else if (point.qualifier() == mscl::MipTypes::CH_Y)
                {
                    m_filter_data.linear_accel_y = point.as_float();
                }
                else if (point.qualifier() == mscl::MipTypes::CH_Z)
                {
                    m_filter_data.linear_accel_z = point.as_float();
                }
            } break;

            /*
            case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER:
            {
                if(point.qualifier() == mscl::MipTypes::CH_ROLL)
                {
                    // m_curr_filter_att_uncert_roll                 = point.as_float();
                    // m_filter_msg.pose.covariance[21]              = pow(m_curr_filter_att_uncert_roll, 2);
                    // m_filtered_imu_msg.orientation_covariance[0]  = m_filter_msg.pose.covariance[21];
                    // m_filter_relative_pos_msg.pose.covariance[21] = m_filter_msg.pose.covariance[21];
                }
                else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
                {
                    // m_curr_filter_att_uncert_pitch                = point.as_float();
                    // m_filter_msg.pose.covariance[28]              = pow(m_curr_filter_att_uncert_pitch, 2);
                    // m_filtered_imu_msg.orientation_covariance[4]  = m_filter_msg.pose.covariance[28];
                    // m_filter_relative_pos_msg.pose.covariance[28] = m_filter_msg.pose.covariance[28];
                }
                else if(point.qualifier() == mscl::MipTypes::CH_YAW)
                {
                    // m_curr_filter_att_uncert_yaw                  = point.as_float();
                    // m_filter_msg.pose.covariance[35]              = pow(m_curr_filter_att_uncert_yaw, 2);
                    // m_filtered_imu_msg.orientation_covariance[8]  = m_filter_msg.pose.covariance[35];
                    // m_filter_relative_pos_msg.pose.covariance[35] = m_filter_msg.pose.covariance[35];
                }
            } break;
             */

            /*
            case mscl::MipTypes::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE:
            {
                if(point.qualifier() == mscl::MipTypes::CH_HEADING)
                {
                    // m_filter_heading_state_msg.heading_rad = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
                {
                    // m_filter_heading_state_msg.heading_uncertainty = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_SOURCE)
                {
                    // m_filter_heading_state_msg.source = point.as_uint16();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
                {
                    // m_filter_heading_state_msg.status_flags = point.as_uint16();
                }
            } break;
            */

            default:
                break;
        }
    }
    //! @note: Then publish the results over moos system
    if(m_moos_node) {
        m_moos_node->publish_filtered(m_filter_data);
    }
}

void Microstrain::parse_imu_packet(const mscl::MipDataPacket &packet) {
    //Handle time
    m_imu_data.time = double(packet.collectedTimestamp().nanoseconds()) ;

    //Data present flags
    bool has_accel = false;
    bool has_gyro  = false;
    bool has_quat  = false;
    bool has_mag   = false;

    //Get the list of data elements
    const mscl::MipDataPoints &points = packet.data();
    
    //Loop over the data elements and map them
    for(const auto& point : points){
        point.field();
    }
    for(const auto& point : points)
    {
        switch(point.field())
        {
            //Scaled Accel
            case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC:
            {
                has_accel = true;
                
                // Stuff into ROS message - acceleration in m/s^2
                if(point.qualifier() == mscl::MipTypes::CH_X)
                {
                    m_imu_data.linear_accel_x = USTRAIN_G * point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Y)
                {
                    m_imu_data.linear_accel_y = USTRAIN_G * point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Z)
                {
                    m_imu_data.linear_accel_z = USTRAIN_G * point.as_float();
                }
            }break;

                //Scaled Gyro
            case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC:
            {
                has_gyro = true;

                if(point.qualifier() == mscl::MipTypes::CH_X)
                {
                    m_imu_data.angular_vel_x = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Y)
                {
                    m_imu_data.angular_vel_y = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Z)
                {
                    m_imu_data.angular_vel_z = point.as_float();
                }
            }break;

            //Scaled Mag
            case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC:
            {
                has_mag = true;

                if(point.qualifier() == mscl::MipTypes::CH_X)
                {
                    m_imu_data.mag_x = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Y)
                {
                    m_imu_data.mag_y = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_Z)
                {
                    m_imu_data.mag_z = point.as_float();
                }
            }break;

            //Scaled Mag
            case mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES:
            {
                has_mag = true;

                if(point.qualifier() == mscl::MipTypes::CH_ROLL)
                {
                    m_imu_data.roll = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
                {
                    m_imu_data.pitch = point.as_float();
                }
                else if(point.qualifier() == mscl::MipTypes::CH_YAW)
                {
                    m_imu_data.yaw = point.as_float();
                }
            }break;

            //Orientation Quaternion
            case mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION:
            {
                has_quat = true;
                
                if(point.qualifier() == mscl::MipTypes::CH_QUATERNION) {
                    mscl::Vector quaternion = point.as_Vector();

                    m_imu_data.quat_w = quaternion.as_floatAt(0);
                    m_imu_data.quat_x = quaternion.as_floatAt(1);
                    m_imu_data.quat_y = quaternion.as_floatAt(2);
                    m_imu_data.quat_z = quaternion.as_floatAt(3);
                }
            }break;

            default:
                break;
        }
    }

    if(has_accel)
    {
        // Since the sensor does not produce a covariance for linear acceleration, set it based on our pulled in parameters.
        // std::copy(m_imu_linear_cov.begin(), m_imu_linear_cov.end(), m_imu_msg.linear_acceleration_covariance.begin());
    }

    if(has_gyro)
    {
        // Since the sensor does not produce a covariance for angular velocity, set it based on our pulled in parameters.
        // std::copy(m_imu_angular_cov.begin(), m_imu_angular_cov.end(), m_imu_msg.angular_velocity_covariance.begin());
    }

    if(has_quat)
    {
        //Since the MIP_AHRS data does not contain uncertainty values we have to set them based on the parameter values.
        // std::copy(m_imu_orientation_cov.begin(), m_imu_orientation_cov.end(), m_imu_msg.orientation_covariance.begin());
    }

    //! @note: Then publish the results over moos system
    if(m_moos_node) {
        m_moos_node->publish_imu(m_imu_data);
    }
}

void Microstrain::run() {
    while(!m_stop) {
        auto now = std::chrono::steady_clock::now();
        auto next = now + std::chrono::milliseconds(long(1000 / m_rate));
        run_once();
        std::this_thread::sleep_until(next);
    }
}

void Microstrain::run_once() {
    auto packets = m_dev->getDataPackets(1000);
    for(const auto& packet : packets) {
        parse_mip_packet(packet);
    }
}

void Microstrain::stop() {
    m_stop = false;
}

void Microstrain::calibrate() {
    // todo: not implemented yet
}

bool Microstrain::test_connection() {
    return m_dev->ping();
}

imu_data_t & Microstrain::get_imu_data() {
    return m_imu_data;
}

imu_data_t & Microstrain::get_filter_data() {
    return m_filter_data;
}

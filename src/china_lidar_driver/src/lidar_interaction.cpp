#include "china_lidar_driver/lidar_interaction.hpp"

// include workarounds
#include "china_lidar_driver/conflict_workaround.hpp"

#include <linux/serial.h>
//#include <termios.h>
#include <asm/termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#define USE_MATH_DEFINES
#include <math.h>

namespace
{
    // define frame values
    constexpr uint8_t KSYNC0 = 0xAA;
    constexpr uint8_t KSYNC1 = 0x55;
}

namespace ChinaLidar
{

    struct PacketHeader
    {
        uint8_t type { 0 };
        uint8_t data_length { 0 };
        uint16_t start_angle { 0 };
        uint16_t end_angle { 0 };
    };

    LidarInteraction::LidarInteraction(PCCallback point_cloud_cb, ScanCallback scan_cb
        , const std::string& new_port, uint32_t new_baud)
        : baudrate(new_baud)
        , port(new_port)
        , raw_data(1024)
        , on_point_cloud_received(point_cloud_cb)
        , on_scan_data_received(scan_cb)
    {
        // open serial port
        serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd == -1)
        {
            perror("Error opening serial port");
            printf("Fail 1");
            return;
        }

        // configure serial port
        // TODO: fix baudrate issues
        struct termios2 tty;
        memset(&tty, 0, sizeof(tty));
        if (workaround::ioctl(serial_fd, TCGETS2, &tty) != 0) {
            perror("Failed to get serial port attributes");
            printf("Fail 2");
            return;
        }
        // set baud rate (Why POSIX does this in this sketchy way?)
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= BOTHER;
        tty.c_ispeed = baudrate;
        tty.c_ospeed = baudrate;
        
        // Apply the specified settings
        tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver and ignore modem control lines
        tty.c_cflag &= ~PARENB;          // No parity
        tty.c_cflag &= ~CSTOPB;          // 1 stop bit
        tty.c_cflag &= ~CSIZE;           // Clear data size bits
        tty.c_cflag |= CS8;              // 8 data bits

        // Set flow control to none
        tty.c_cflag &= ~CRTSCTS;

        // Apply the changes to the serial port attributes
        if (workaround::ioctl(serial_fd, TCSETS2, &tty) != 0) {
            perror("Failed to set serial port attributes");
            printf("Fail 3");
            return;
        }

        state = State::SYNC0;
    }

    LidarInteraction::~LidarInteraction()
    {
        // close serial port if it's open
        if (serial_fd != -1)
        {
            close(serial_fd);
        }
    }

    void LidarInteraction::parseData()
    {
        bool has_data = false;
        PacketHeader header;

        state = State::SYNC0;
        raw_data.resize(1024);
        memset(raw_data.data(), 0, raw_data.size() * sizeof(uint8_t));
        while (serial_fd != -1 && !has_data)
        {
            switch (state)
            {
            case State::SYNC0:
            {
                if (read(serial_fd, raw_data.data(), 1) == 1)
                {
                    //printf("Data read[0]: %d", raw_data[0]);
                    if (raw_data[0] == KSYNC0)
                    {
                        state = State::SYNC1;
                    }
                }
                break;
            }
            case State::SYNC1:
            {
                if (read(serial_fd, raw_data.data() + 1, 1) == 1)
                {
                    //printf("Data read[1]: %d", raw_data[1]);
                    if (raw_data[1] == KSYNC1)
                    {
                        state = State::HEADER;
                        break;
                    }
                }
                state = State::SYNC0;
                break;
            }
            case State::HEADER:
            {
                read(serial_fd, raw_data.data() + 2, 8);
                header.type = raw_data[2];
                header.data_length = raw_data[3];
                // data is little endian
                header.start_angle = (raw_data[5] << 8) | raw_data[4];
                header.end_angle = (raw_data[7] << 8) | raw_data[6];
                //TODO: check if data is valid (CRC is located at the end of the packet)
                state = State::DATA;
            }
            case State::DATA:
            {
                uint16_t data_length = header.data_length * 3;
                read(serial_fd, raw_data.data() + 10, data_length);
                // this part is ported from python script posted on Vidcon discord server
                // not sure why python code that just visualizes the data was doing this
                //printf("header.type = %d\n", header.type);
                
                if (header.type & 1)
                {
                    if(on_point_cloud_received)
                    {
                        on_point_cloud_received(point_cloud_data);
                    }
                    point_cloud_data.clear();
                    //return;
                }

                int16_t angle_diff = header.end_angle - header.start_angle;
                if(header.end_angle < header.start_angle)
                {
				    angle_diff =  0xB400 - header.start_angle + header.end_angle;
                }

                int16_t angle_per_sample = 0;
                if(angle_diff > 1)
                {
                    if (header.data_length - 1 < 1)
                    {
                        angle_per_sample = 1;
                        return;
                    }
                    else
                    {
                        angle_per_sample = angle_diff / (header.data_length - 1);
                    }
                }

                for (size_t i = 0; i < header.data_length; i++)
                {
                    const uint16_t index = 10 + (i * 3) ;
                    const uint8_t intensity = raw_data[index + 0];
                    const uint8_t distanceL = raw_data[index + 1];
                    const uint8_t distanceH = raw_data[index + 2];
                    // get real distance value
                    const uint16_t distance = static_cast<uint16_t>(distanceH << 8) 
                                                + static_cast<uint16_t>(distanceL);
                    
                    const float distance_meters = static_cast<float>(distance) / 4000.0f;
                    const float step = M_PI * 2.0f;
                    
                    auto angle = static_cast<float>(header.start_angle + (i * angle_per_sample));
                    angle = (step * angle) / static_cast<float>(0xB400);

                    if (angle < last_angle)
                    {
                        if(on_scan_data_received)
                        {
                            on_scan_data_received(scan_data);
                        }
                        scan_data.clear();
                    }
                    last_angle = angle;

                    // fill vectors with data
                    // Filling point cloud with data
                    const float angle_inv = (M_PI * 2.0f) - angle;
                    Point2D point;
                    point.x = std::cos(angle_inv) * distance_meters;
                    point.y = std::sin(angle_inv) * distance_meters;
                    point.intensity = intensity;
                    point_cloud_data.push_back(point);
                    // Filling scan data with data
                    Vector2D scan_point;
                    scan_point.distance = distance_meters;
                    scan_point.angle = angle;
                    scan_point.intensity = intensity;
                    scan_data.push_back(scan_point);
                }

                has_data = true;
                state = State::SYNC0;
            }
            default:
                break;
            }
        }
    }
}
#pragma once

// Ported from https://megauti.blogspot.com/2021/08/lidar-radarmb1r2tv158-or-v159-program.html

#include <string>
#include <stdint.h>
#include <vector>
#include <functional>

namespace ChinaLidar
{

struct Point2D
{
    float x { 0.0f };
    float y { 0.0f };
    int8_t intensity { 0 };
};

struct Vector2D
{
    float distance { 0.0f };
    float angle { 0.0f };
    int8_t intensity { 0 };
};

class LidarInteraction
{
    typedef std::function<void(const std::vector<Point2D>&)> PCCallback;
    typedef std::function<void(const std::vector<Vector2D>&)> ScanCallback;
public:

    enum class State
    {
        SYNC0 = 0,
        SYNC1 = 1,
        HEADER = 2,
        DATA = 3
    };

    LidarInteraction(PCCallback point_cloud_cb, ScanCallback scan_cb
        , const std::string& port, uint32_t baudrate);
    ~LidarInteraction();

    /// @brief parses serial data so that we can get point and scan data
    void parseData();

    /// @brief Returns true if the scan and point data is ready
    /// @return true if data is ready
    inline bool isDataAvailable() const { return data_ready; }

private:
    bool data_ready { false };
    float last_angle { 0.0f };
    State state { State::SYNC0 };
    uint32_t baudrate { 153600 };
    std::string port;
    int serial_fd { -1 };
    // variable to store read data
    std::vector<uint8_t> raw_data;

    std::vector<Point2D> point_cloud_data;
    std::vector<Vector2D> scan_data;

    PCCallback on_point_cloud_received;
    ScanCallback on_scan_data_received;
};

}
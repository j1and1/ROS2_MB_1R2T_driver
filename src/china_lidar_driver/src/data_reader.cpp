#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "china_lidar_driver/lidar_interaction.hpp"
#include "china_lidar_driver/point_cloud_data_structs.hpp"

#include <vector>
#include <string>

namespace
{
    // serial port related constants
    constexpr char serial_port_key[] = "device";
    constexpr char baud_rate_key[] = "baud_rate";
    // publisher related constants
    constexpr char frame_id_key[] = "frame_id";
    constexpr char scan_topic_key[] = "scan";
    constexpr char pc_topic_key[] = "cloud";
}

class DataReader : public rclcpp::Node 
{
    typedef sensor_msgs::msg::PointCloud2 PC;
    typedef sensor_msgs::msg::LaserScan LS;
public:

    DataReader(): Node("data_reader")
    {
        RCLCPP_INFO(get_logger(), "Data reader node is starting...");
        // declare parameters
        declare_parameter<int>(baud_rate_key, 153600);
        declare_parameter<std::string>(serial_port_key, "/dev/ttyUSB0");
        declare_parameter<std::string>(frame_id_key, "laser");

        // create publishers... history size is 10 elements
        pointcloud_publisher = create_publisher<PC>(pc_topic_key, 10);
        scan_publisher = create_publisher<LS>(scan_topic_key, 10);

        //
        clock = std::make_shared<rclcpp::Clock>();
        initialize_msgs();

        // create instance of LidarInteraction class
        RCLCPP_INFO(get_logger(), "Initializing LidarInteraction class...");
        auto pc_callback = std::bind(&DataReader::createPointCloudMessage, this, std::placeholders::_1);
        auto ls_callback = std::bind(&DataReader::createLaserScanMessage, this, std::placeholders::_1);
        lidar_interaction.reset(new ChinaLidar::LidarInteraction(
            pc_callback, ls_callback,
            get_parameter(serial_port_key).as_string(),
            get_parameter(baud_rate_key).as_int()));

        RCLCPP_INFO(get_logger(), "Data reader node is started...");
    }

    void runLidar()
    {
        lidar_interaction->parseData();
    }

private:
    rclcpp::Publisher<PC>::SharedPtr pointcloud_publisher;
    rclcpp::Publisher<LS>::SharedPtr scan_publisher;
    std::shared_ptr<ChinaLidar::LidarInteraction> lidar_interaction;
    rclcpp::Clock::SharedPtr clock;

    PC pc_msg_base;
    LS ls_msg_base;
    
    void createPointCloudMessage(const std::vector<ChinaLidar::Point2D>& points)
    {
        // if no data available return
        if (points.size() == 0)
        {
            return;
        }

        PC pc_msg = pc_msg_base;
        pc_msg.header.stamp = clock->now();
        pc_msg.width = points.size();
        // Fill the message with point cloud data
        for(size_t i = 0; i < points.size(); i++) 
        {
            const ChinaLidar::Point2D& p = points[i];
            Point_u data;
            data.point_data.x = p.x;
            data.point_data.y = p.y;
            data.point_data.z = 0.0;
            data.point_data.intensity = p.intensity;
            // fill message with byte data of each point
            for (size_t i2 = 0; i2 < pc_msg.point_step; i2++)
            {
                pc_msg.data.push_back(data.bytes[i2]);
            }
        }
        
        pointcloud_publisher->publish(pc_msg);
    }

    void createLaserScanMessage(const std::vector<ChinaLidar::Vector2D>& points)
    {
        // if no data available return
        if (points.size() == 0)
        {
            return;
        }
        
        // take copy of the original message
        LS scan_msg = ls_msg_base;
        scan_msg.header.stamp = clock->now();
        // need to calculate this from points that are in scan
        scan_msg.angle_increment = 2 * M_PI / points.size();
        scan_msg.time_increment = 0.2 / points.size(); // TODO: fix this
        scan_msg.scan_time = 0.2; //TODO: not the real value
        
        for(size_t i = 0; i < points.size(); i++) 
        { 
            const ChinaLidar::Vector2D& vector = points.at((points.size()-1) - i);
            scan_msg.ranges.push_back(vector.distance);
            scan_msg.intensities.push_back(vector.intensity);
        }
        
        scan_publisher->publish(scan_msg);
    }

    /// @brief Initializes scan and point cloud messages so we dont have to do that on each callback call.
    void initialize_msgs()
    {
        std::string frame_id;
        get_parameter(frame_id_key, frame_id);

        // 
        LS& scan = ls_msg_base;
        scan.header.frame_id = frame_id;
        scan.range_min = 0.0;
        scan.angle_max = 2.0 * M_PI;
        scan.range_min = 0.11;
        scan.range_max = 8.0;
        //  
        PC& pc = pc_msg_base;
        pc.header.frame_id = frame_id;
        pc.header.stamp = clock->now();
        pc.is_bigendian = false;
        pc.is_dense = false;
        pc.height = 1; 
        pc.point_step = 13; // Float32 = 4 byte * 3(xyz) + 1 byte intensity = 13 bytes

        // need to find the best way to do this as this seems sub optimal...
        sensor_msgs::msg::PointField field;
        field.name = "x";
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        field.offset = 0;
        pc.fields.push_back(field);

        field.name = "y";
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        field.offset = 4;
        pc.fields.push_back(field);

        field.name = "z";
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        field.offset = 8;
        pc.fields.push_back(field);

        field.name = "intensity";
        field.datatype = sensor_msgs::msg::PointField::UINT8;
        field.count = 1;
        field.offset = 12;
        pc.fields.push_back(field);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataReader>();
    while(rclcpp::ok())
    {
        node->runLidar();
        rclcpp::spin_some(node);
    }
    //rclcpp::spin(std::make_shared<DataReader>());
    rclcpp::shutdown();
    return 0;
}
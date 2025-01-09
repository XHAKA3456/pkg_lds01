#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class NodeLds01 : public rclcpp::Node
{
    public:

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_laser;

        
        sensor_msgs::msg::LaserScan::SharedPtr msg_laser;

        boost::asio::io_service loop;
        boost::asio::serial_port serial_laser;


        // boost::array<uint8_t, 2520> raw_bytes;
        uint8_t start_count;
        bool shutting_down_;
        bool got_scan;
        uint8_t good_sets;
        uint32_t motor_speed;
        uint16_t rpms;
        int index;



        tf2_ros::TransformBroadcaster tf_broadcaster_;



        NodeLds01();
        ~NodeLds01();

        void poll();
        void publishTransform();
        

};
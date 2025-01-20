#include "pkg_lds01/lds01.hpp"

NodeLds01::NodeLds01()
: Node("node_lds01"), 
  serial_laser(loop,"/dev/ttyUSB2"), 
  tf_broadcaster_(this) 
{
    // QoS 설정 수정: rclcpp::SensorDataQoS로 기본값 설정
    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    
    // 신뢰성 있는 메시지 전송을 위해 Reliable 설정 추가
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);

    publisher_laser = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);
    msg_laser = std::make_shared<sensor_msgs::msg::LaserScan>();

    serial_laser.set_option(boost::asio::serial_port_base::baud_rate(230400));

    start_count = 0;
    shutting_down_ = false;
    got_scan = false;
    good_sets = 0;
    motor_speed = 0;
    rpms = 0;
    index = 0;

    boost::asio::write(serial_laser, boost::asio::buffer("b", 1));  // start motor
    RCLCPP_INFO(this->get_logger(),"motor start!!");
}

NodeLds01::~NodeLds01()
{
    boost::asio::write(serial_laser, boost::asio::buffer("e", 1));  // stop motor
    shutting_down_ = true;
}


void NodeLds01::poll(){

    shutting_down_ = false;
    got_scan = false;

    boost::array<uint8_t, 2520> raw_bytes;
    start_count = 0;
    good_sets = 0;
    motor_speed = 0;
    rpms = 0;

    while( !shutting_down_ && !got_scan ){

        try{
            boost::asio::read( serial_laser, boost::asio::buffer(&raw_bytes[start_count],1) );    
        }catch(const boost::system::system_error& e){
            RCLCPP_ERROR( this->get_logger(),"failed to read from serial port: %s",e.what() );
            return;
        }

        if(start_count == 0){
            if(raw_bytes[start_count] == 0xFA){
                start_count = 1;
            }
        }
        else if(start_count == 1){
            if(raw_bytes[start_count] == 0xA0){
                start_count = 0;

                got_scan = true;

                boost::asio::read(serial_laser,boost::asio::buffer(&raw_bytes[2], 2518));

                msg_laser->angle_increment = (2.0*M_PI/360.0);
                msg_laser->angle_min = 0.0;
                msg_laser->angle_max = 2.0*M_PI-msg_laser->angle_increment;
                msg_laser->range_min = 0.12;
                msg_laser->range_max = 3.5;
                msg_laser->ranges.resize(360);
                msg_laser->intensities.resize(360);

                //read data in sets of 6
                for(uint16_t i = 0; i < raw_bytes.size(); i=i+42)
                {
                    if(raw_bytes[i] == 0xFA && raw_bytes[i+1] == (0xA0 + i / 42)) //&& CRC check
                    {
                        good_sets++;
                        motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2]; //accumulate count for avg. time increment

                        for(uint16_t j = i+4; j < i+40; j=j+6)
                        {
                        index = 6*(i/42) + (j-4-i)/6;

                        // Four bytes per reading
                        uint8_t byte0 = raw_bytes[j];
                        uint8_t byte1 = raw_bytes[j+1];
                        uint8_t byte2 = raw_bytes[j+2];
                        uint8_t byte3 = raw_bytes[j+3];

                        uint16_t intensity = (byte1 << 8) + byte0;
                        uint16_t range = (byte3 << 8) + byte2;

                        msg_laser->ranges[359-index] = range / 1000.0;
                        msg_laser->intensities[359-index] = intensity;
                        }
                    }
                }
                rpms=motor_speed / good_sets / 10;
                msg_laser->time_increment = (float)(1.0 / (rpms*6));
                msg_laser->scan_time = msg_laser->time_increment * 360;                
            }
            else{
                start_count = 0;
            }
        }

    }

}


void NodeLds01::publishTransform(){

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "chassis_link";
    transform.child_frame_id = "laser";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(transform);
}


int main(int argc, char **argv){


    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeLds01>();
    
    while( rclcpp::ok() ){

        node->msg_laser->header.frame_id = "laser";
        node->poll();
        node->msg_laser->header.stamp = node->now();

        node->publishTransform();

        node->publisher_laser->publish(*node->msg_laser);
    }

    node->shutting_down_= true;


    return 0;
}

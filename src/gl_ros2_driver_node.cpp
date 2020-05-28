#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "gl_driver.h"


using namespace std::chrono_literals;


class GlRos2DriverNode : public rclcpp::Node
{
    public:
        GlRos2DriverNode() : Node("gl_ros2_driver_node")
        {
            DeclareParam();
            GetParam();
            
            laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topicname_lidar, 10);
            timer = this->create_wall_timer(25ms, std::bind(&GlRos2DriverNode::TimerCallback, this));

            InitGl();
        }

        ~GlRos2DriverNode()
        {
            gl.SetFrameDataEnable(false);
        }


    private:
        /************************** Functions *****************************/
        void TimerCallback(void);
        void DeclareParam(void);
        void GetParam(void);

        void InitGl(void);
        void PubLidar(Gl::framedata_t frame_data);


    private:
        /************************** Launch variables *****************************/
        std::string serial_port_name = "/dev/ttyUSB0";
        std::string frame_id = "laser";
        std::string pub_topicname_lidar = "scan";
        

    private:
        /************************** Other variables *****************************/
        Gl gl = Gl();
        
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
        rclcpp::TimerBase::SharedPtr timer;
};

void GlRos2DriverNode::TimerCallback(void)
{
    Gl::framedata_t frame_data = gl.ReadFrameData();
    if(frame_data.distance.size()>0) PubLidar(frame_data);
}

void GlRos2DriverNode::DeclareParam(void)
{
    this->declare_parameter("serial_port_name", serial_port_name);
    this->declare_parameter("frame_id", frame_id);
    this->declare_parameter("pub_topicname_lidar", pub_topicname_lidar);
}

void GlRos2DriverNode::GetParam(void)
{
    serial_port_name = this->get_parameter("serial_port_name").as_string();
    frame_id = this->get_parameter("frame_id").as_string();
    pub_topicname_lidar = this->get_parameter("pub_topicname_lidar").as_string();
}

void GlRos2DriverNode::InitGl(void)
{
    gl.OpenSerial(serial_port_name,921600);
    gl.SetFrameDataEnable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "Serial Num : " << gl.GetSerialNum() << std::endl;
    gl.SetFrameDataEnable(true);
}

void GlRos2DriverNode::PubLidar(Gl::framedata_t frame_data) 
{
    int num_lidar_data = frame_data.distance.size();
    
    if(num_lidar_data>0)
    {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = rclcpp::Clock().now();

        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_max = frame_data.angle[0];
        scan_msg.angle_min = frame_data.angle[num_lidar_data-1];
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(num_lidar_data-1);
        scan_msg.time_increment = scan_msg.scan_time / (double)(num_lidar_data-1);
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 30.0;
        scan_msg.ranges.resize(num_lidar_data);
        for(size_t i=0; i<num_lidar_data; i++)
        {
            scan_msg.ranges[i] = (double)frame_data.distance[i];
        }
        laser_pub->publish(scan_msg);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlRos2DriverNode>());
    rclcpp::shutdown();
   
    return 0;
}

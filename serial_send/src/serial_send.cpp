#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stair_info_msg/stair_info.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define    sBUFFERSIZE    19//send buffer size 串口发送缓存长度
#define    rBUFFERSIZE    27//receive buffer size 串口接收缓存长度

serial::Serial ser;

//取消字节对齐
#pragma pack (1)
union Package
{
    struct
    {
        uint8_t head;
        uint8_t flag;
        float v_depth;
        float v_height;
        float depth;
        float height;
        uint8_t tail;
    };
    uint8_t data[19];
};
#pragma pack ()

void data_pack(const stair_info_msg::stair_info &msgs, Package &pack)
{
    pack.head = 0xFF;
    pack.flag = static_cast<bool>(msgs.has_stair);
    pack.height = msgs.height;
    pack.depth = msgs.depth;
    pack.v_height = msgs.v_height;
    pack.v_depth = msgs.v_depth;
    pack.tail = 0xFE;
}


void callback(const stair_info_msg::stair_info &msgs)
{
    Package pack;
    data_pack(msgs, pack);

    /*
	for(i=0;i<15;i++){
		ROS_INFO("0x%02x",pack.data[i]);
	}
	*/
    ser.write(pack.data, sBUFFERSIZE);
    ROS_INFO("robot_serial_port has send message, height: %.2f, depth: %.2f, v_height: %.2f, v_depth: %.2f",
             msgs.height, msgs.depth, msgs.v_height, msgs.v_depth);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_node");
    ros::NodeHandle nh("~");
    ros::Subscriber sub;
    int queue_size = 5;
    std::string robot_serial_port = "/dev/ttyUSB0";

    nh.getParam("queuesize", queue_size);
    nh.getParam("robot_serial_port", robot_serial_port);

    try
    {
        ser.setPort(robot_serial_port);
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open robot port : " + robot_serial_port);
        return -1;
    }

    if (ser.isOpen())
    {
        sub = nh.subscribe("stair_info", queue_size, callback);
        ROS_INFO_STREAM("Serial Port initialized");
    } else
    {
        return -1;
    }
    ros::spin();
    return 0;
}


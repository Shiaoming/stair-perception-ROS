//
// Created by zxm on 18-7-23.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "imu_reader/serial.h"
#include "imu_reader/packet.h"
#include "imu_reader/imu_data_decode.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_reader");
    ros::NodeHandle n;

    std::string serial_name;

    if(argc == 2)
    {
        serial_name = argv[1];
    } else{
        n.getParam("serial_port", serial_name);
    }

    if(serial_name.empty())
    {
        ROS_ERROR("Please set serial name");
        return -1;
    }

    if(open_imu(serial_name.c_str()) != -1)
    {
        ROS_INFO("Successfully open serial port: %s", serial_name.c_str());
        ROS_INFO("Print IMU data every 1 sec.");
    }
    else{
        ROS_ERROR("Cannot open serial port: %s", serial_name.c_str());
        return -1;
    }

    

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 2);

    ros::Rate loop_rate(150);

    int16_t Acc[3];
    int16_t Gyo[3];
    int16_t Mag[3];
    float Euler[3];
    float Quat[4];
    uint32_t count=0;

    float last_q0;

    while (ros::ok())
    {

        sensor_msgs::Imu imu_msg;

        read_once(Acc, Gyo, Mag, Euler, Quat);

        if(Quat[0] != last_q0)
        {
            imu_msg.header.frame_id = "imu_pose";
            imu_msg.header.seq = count++;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.orientation.w = Quat[0];
            imu_msg.orientation.x = Quat[1];
            imu_msg.orientation.y = Quat[2];
            imu_msg.orientation.z = Quat[3];
            imu_msg.angular_velocity.x = Gyo[0];
            imu_msg.angular_velocity.y = Gyo[1];
            imu_msg.angular_velocity.z = Gyo[2];
            imu_msg.linear_acceleration.x = Acc[0];
            imu_msg.linear_acceleration.y = Acc[1];
            imu_msg.linear_acceleration.z = Acc[2];

            imu_pub.publish(imu_msg);

            last_q0 = Quat[0];
        }

//        if(count % 100 == 0)
//            ROS_INFO("IMU data: Q_wxyz(%4f,%4f,%4f,%4f)", Quat[0], Quat[1], Quat[2], Quat[3]);

        ros::spinOnce();

        loop_rate.sleep();
    }

    close_imu();

    ROS_INFO("IMU closed, exiting...");

    return 0;
}
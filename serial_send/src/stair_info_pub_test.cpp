//
// Created by zxm on 18-8-20.
//

#include <ros/ros.h>
#include <string>
#include "stair_info_msg/stair_info.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stair_info_pub_test");
    ros::NodeHandle nh("~");
    int queue_size = 5;
    int rate = 10;
    bool has_stair = false;
    float height = 100;
    float depth = 200;
    float v_height = 1500;
    float v_depth = 1200;

    nh.getParam("queue_size", queue_size);
    nh.getParam("loop_rate", rate);
    nh.getParam("has_stair", has_stair);
    nh.getParam("height", height);
    nh.getParam("depth", depth);
    nh.getParam("v_height", v_height);
    nh.getParam("v_depth", v_depth);

    ros::Publisher pub = nh.advertise<stair_info_msg::stair_info>("stair_info", queue_size);

    stair_info_msg::stair_info msg;
    msg.has_stair = has_stair;
    msg.height = height;
    msg.depth = depth;
    msg.v_height = v_height;
    msg.v_depth = v_depth;

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
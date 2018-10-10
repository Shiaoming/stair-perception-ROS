//
// Created by zxm on 18-7-31.
//

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <string>

using namespace std;

cv_bridge::CvImagePtr cv_ptr;
cv::Mat image;
pcl::PointCloud<pcl::PointXYZRGBA> cloud;
bool new_cloud = false;
bool save = false;
bool exitflag = false;

void callback(const sensor_msgs::PointCloud2::ConstPtr cloud_ros)
{
    pcl::fromROSMsg(*cloud_ros, cloud);

    for (auto &p:cloud.points)
    {
        if (!pcl_isfinite (p.x) ||
            !pcl_isfinite (p.y) ||
            !pcl_isfinite (p.z))
        {
//            ROS_INFO("pcl_isinfinite!");
            p.x = p.y = p.z = 0;
            continue;
        }
    }

    new_cloud = true;

//    pcl::PCDWriter writer;
//    writer.write("test.pcd", cloud);
}

void callbackimg(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.keyDown())
    {
        if (event.getKeyCode() == 's')
        {
            save = true;
        } else if (event.getKeyCode() == 'q' || event.getKeyCode() == 27)
        {
            exitflag = true;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_viewer");
    ros::NodeHandle private_nh("~");

    string topic_name = "/buxx_cloud_rotation/cloud_xyzrgba";
    string topic_name_image = "/buxx/qhd/image_color";

    ros::Subscriber sub, subimg;

    sub = private_nh.subscribe(topic_name, 5, callback);
    subimg = private_nh.subscribe(topic_name_image, 5, callbackimg);

    cv::namedWindow("image", cv::WINDOW_NORMAL);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("cloud"));
    viewer->setPosition(0, 0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(0.3);
    viewer->setCameraPosition(-0.596784, -2.03596, 2.79617, -0.949447, 0.215143, -0.228612);
    viewer->setPosition(100,100);
    viewer->setSize(780,540);

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) viewer.get());

    ros::Rate loop_rate(30);
    bool first = true;

    while (ros::ok())
    {
        if (new_cloud)
        {
            new_cloud = false;
            if (first)
            {
                first = false;
                pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> color_handler(
                        cloud.makeShared());
                viewer->addPointCloud<pcl::PointXYZRGBA>(cloud.makeShared(), color_handler, "cloud", 0);
            } else
            {
                pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> color_handler(
                        cloud.makeShared());
                viewer->updatePointCloud<pcl::PointXYZRGBA>(cloud.makeShared(), color_handler, "cloud");
            }

            if(!image.empty())
                cv::imshow("image", image);
        }

        char ch = cv::waitKey(3);

        if (ch == 's')
            save = true;
        else if (ch == 'q' || ch == 27)
            exitflag = true;

        if (save)
        {
            save = false;

//            time_t t(cloud.header.stamp/1000000);
//            struct tm *tt;
//            tt = localtime(&t);
//
//            char ch[100]={0};
//            std::sprintf(ch, "%04d%02d%02d-%02d%02d%02d-%03d",
//                         tt->tm_year + 1900,
//                         tt->tm_mon + 1,
//                         tt->tm_mday,
//                         tt->tm_hour,
//                         tt->tm_min,
//                         tt->tm_sec,
//                         cloud.header.stamp%1000000/1000);

            char ch[100]={0};
            std::sprintf(ch, "%d.%06d",
                         cloud.header.stamp/1000000,
                         cloud.header.stamp%1000000);

            std::string file_name=ch;

            ROS_INFO("Saving with name: %s", ch);

            pcl::PCDWriter writer;
            writer.write(file_name + ".pcd", cloud);

            if(!image.empty())
                cv::imwrite(file_name + ".png", image);
        }

        if (exitflag)
            break;

        viewer->spinOnce(30);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
//
// Created by zxm on 18-8-18.
//

//
// Created by zxm on 18-8-8.
//
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "plane_msg/Plane.h"
#include "plane_msg/VecPlane.h"
#include "pcl_segmentation/PCLSeg.h"

#include <thread>
#include <mutex>
#include <chrono>

#include <pcl_segmentation/pclseg_paramConfig.h>


using namespace std;
using namespace std::chrono;

class PCLSegNode
{
public:
    explicit PCLSegNode(const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
                        const std::string &topicCloudin = "cloud_in",
                        const std::string &topicVecPlane = "vec_planes") :
            private_nh(private_nh), queueSize(5), count(0), running(true), show_viewer(true),
            topicCloudin(topicCloudin), topicVecPlane(topicVecPlane),
            server(new dynamic_reconfigure::Server<pcl_segmentation::pclseg_paramConfig>(private_nh)) {}

    ~PCLSegNode()
    {
        delete server;
        running = false;
        if (show_viewer)
            viz_thread.join();
    }

    void init()
    {
        ROS_INFO("############# PCLSeg start #############");

        sub = private_nh.subscribe(topicCloudin, queueSize, &PCLSegNode::callback, this);

        f = boost::bind(&PCLSegNode::cfg_callback, this, _1, _2);
        server->setCallback(f);

        pub_vec_planes = private_nh.advertise<plane_msg::VecPlane>(topicVecPlane, queueSize);

        private_nh.getParam("show_seg_viewer", show_viewer);

        ROS_INFO("show_seg_viewer: %d", show_viewer);

        time_record.open("pclseg_time_record.csv");
        time_record << "TIME_STAMP" << "," << "normalEstimation" << "," << "passThrough"
                << "," << "voxelGrid"<< "," << "remove&classify"<< "," << "segmentHorizontalPlanes"
                << "," << "segmentVerticalPlanes"<< endl;

        if (show_viewer)
        {
            viz_thread = thread(&PCLSegNode::rviz_loop, this);
        }
    }

    void cfg_callback(pcl_segmentation::pclseg_paramConfig &config, uint32_t level)
    {
        plane_seg.x_min = config.x_min;
        plane_seg.x_max = config.x_max;
        plane_seg.y_min = config.y_min;
        plane_seg.y_max = config.y_max;
        plane_seg.z_min = config.z_min;
        plane_seg.z_max = config.z_max;

        plane_seg.normalEstimationMethod = pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::NormalEstimationMethod(
                config.normalEstimationMethod);

        plane_seg.depth_change_factor = config.depth_change_factor;
        plane_seg.normal_smoothing_size = config.normal_smoothing_size;

        plane_seg.voxel_x = config.voxel_x;
        plane_seg.voxel_y = config.voxel_y;
        plane_seg.voxel_z = config.voxel_z;

        plane_seg.parallel_angle_diff = config.parallel_angle_diff;
        plane_seg.perpendicular_angle_diff = config.perpendicular_angle_diff;

        plane_seg.noleg_distance = config.noleg_distance;
        plane_seg.seg_threshold = config.seg_threshold;
        plane_seg.seg_plane_angle_diff = config.seg_plane_angle_diff;

        plane_seg.seg_rest_point = config.seg_rest_point;
        plane_seg.seg_max_iters = config.seg_max_iters;
        plane_seg.cluster_tolerance = config.cluster_tolerance;
        plane_seg.min_cluster_size = config.min_cluster_size;
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr cloud_ros)
    {
        count++;
        std::vector<double> vectime;

        pcl::fromROSMsg(*cloud_ros, cloud_in_pcl_xyzrgba);

        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> vec_planes;

        auto t1 = system_clock::now();

        plane_seg.process(cloud_in_pcl_xyzrgba, vec_planes, vectime);

        auto t2 = system_clock::now();
        auto duration = duration_cast<microseconds>(t2 - t1);
        process_time = duration.count();
        ROS_INFO("Plane segmentation time: %.2f ms", process_time / 1000.0);

        // publish segmented planes
        plane_msg::VecPlane vecPlane;
        GenerateVecPlanes(vec_planes, count, vecPlane, cloud_ros->header.stamp);
        pub_vec_planes.publish(vecPlane);

        mutex_vsp.lock();
        vsp_plane_viz = vec_planes;
        mutex_vsp.unlock();

        time_record << cloud_ros->header.stamp << ",";
        for (auto &t : vectime)
            time_record << t << ",";
        time_record << std::endl;
    }

    void GenerateVecPlanes(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> &vec_planes,
                           const unsigned long count, plane_msg::VecPlane &vecPlane,
                           const ros::Time &stamp)
    {
        int plane_number = vec_planes.size();
        vecPlane.vecPlane.resize(plane_number);

        vecPlane.header.seq = count;
        vecPlane.header.stamp = stamp;
        vecPlane.header.frame_id = "imu_pose";

        // plane information
        for (size_t i = 0; i < plane_number; i++)
        {
            pcl::toROSMsg(vec_planes[i], vecPlane.vecPlane[i].cloud);
            vecPlane.vecPlane[i].cloud.header = vecPlane.header;
        }
    }

    void rviz_loop()
    {
        viewer.reset(new pcl::visualization::PCLVisualizer("seg_viewer"));
        viewer->setPosition(0, 0);
        viewer->initCameraParameters();
        viewer->setBackgroundColor(1, 1, 1);
        viewer->addCoordinateSystem(0.3);
        viewer->setCameraPosition(-0.596784, -2.03596, 2.79617, -0.949447, 0.215143, -0.228612);
        viewer->setPosition(100,100);
        viewer->setSize(780,540);
        while (running)
        {
            std::string viewr_cloud_name = "cloud_";

            mutex_vsp.lock();
            if (!vsp_plane_viz.empty())
            {
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                srand(0);
                for (unsigned int i = 0; i < vsp_plane_viz.size(); i++)
                {

                    double r, g, b;
                    r = int(255.0 * rand() / (RAND_MAX + 1.0));
                    g = int(255.0 * rand() / (RAND_MAX + 1.0));
                    b = int(255.0 * rand() / (RAND_MAX + 1.0));
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
                            single_color(vsp_plane_viz[i].makeShared(), r, g, b);

                    std::stringstream ss;

                    ss << viewr_cloud_name << i;

                    if (vsp_plane_viz[i].points.size())
                    {
                        // add cloud
                        ss << "c";
                        viewer->addPointCloud<pcl::PointXYZRGBA>(vsp_plane_viz[i].makeShared(), single_color, ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                                                 ss.str());
                    }
                }
            }
            mutex_vsp.unlock();

            viewer->spinOnce(30);
        }
    }

private:
    std::ofstream time_record;

    /*********** ros related ***********/
    uint32_t queueSize;
    ros::NodeHandle private_nh;
    ros::Subscriber sub;
    ros::Publisher pub_vec_planes;
    std::string topicCloudin, topicVecPlane;

    dynamic_reconfigure::Server<pcl_segmentation::pclseg_paramConfig> *server;
    dynamic_reconfigure::Server<pcl_segmentation::pclseg_paramConfig>::CallbackType f;

    /************ PCLSeg ************/
    PCLSeg plane_seg;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_in_pcl_xyzrgba;

    bool show_viewer;
    thread viz_thread;
    mutex mutex_vsp;
    bool running;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> vsp_plane_viz;

    unsigned long count;
    unsigned long process_time;
};

class PCLSegNodelet : public nodelet::Nodelet
{
public:
    PCLSegNodelet() : Nodelet(), pstair_modeling(nullptr) {}

    ~PCLSegNodelet() override { delete pstair_modeling; }

private:
    void onInit() override
    {
        pstair_modeling = new PCLSegNode(getPrivateNodeHandle());
        pstair_modeling->init();
    }

    PCLSegNode *pstair_modeling;
};

PLUGINLIB_DECLARE_CLASS(PCLSegNode, PCLSegNodelet, PCLSegNodelet, nodelet::Nodelet);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_seg");
    ros::NodeHandle private_nh("~");

    PCLSegNode pclSegNode(private_nh, "/peac_cloud_rotation/cloud_xyzrgba");

    pclSegNode.init();

    ros::spin();

    ros::shutdown();
    return 0;
}

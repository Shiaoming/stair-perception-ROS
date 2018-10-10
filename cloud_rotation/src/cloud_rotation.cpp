//
// Created by zxm on 18-7-31.
//

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/common.h>

#include <math.h> //fabs
#include <sensor_msgs/Imu.h>


class CloudRotation
{
public:
    CloudRotation(const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
                  const std::string &topicPointCloud2 = "cloud_in",
                  const std::string &topicImu = "imu_data") :
            private_nh(private_nh), queueSize(5), useExact(false), count(0),
            topicPointCloud2(topicPointCloud2), topicImu(topicImu),
            topicPointCloudRotXYZRGBA("cloud_xyzrgba"), topicPointCloudRotXYZ("cloud_xyz") {}

    void init()
    {
        subPointCloud2 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(private_nh, topicPointCloud2,
                                                                                   queueSize);
        subImu = new message_filters::Subscriber<sensor_msgs::Imu>(private_nh, topicImu, queueSize);

        if (useExact)
        {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize),
                                                                           *subPointCloud2, *subImu);
            syncExact->registerCallback(boost::bind(&CloudRotation::callback, this, _1, _2));
        } else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(
                    ApproximateSyncPolicy(queueSize), *subPointCloud2, *subImu);
            syncApproximate->registerCallback(boost::bind(&CloudRotation::callback, this, _1, _2));
        }

        pub_xyzrgba = private_nh.advertise<sensor_msgs::PointCloud2>(topicPointCloudRotXYZRGBA, queueSize);
        pub_xyz = private_nh.advertise<sensor_msgs::PointCloud2>(topicPointCloudRotXYZ, queueSize);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr cloud_ros, const sensor_msgs::Imu::ConstPtr imu)
    {
        computeRotationMatrix(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z,
                              rotation_matrix);

        pcl::fromROSMsg(*cloud_ros, cloud_unrot);

        rotationCloudPoints(cloud_unrot, rotation_matrix, cloud_rot, cloud_rot_xyz);

        pcl::toROSMsg(cloud_rot, cloud_ros_xyzrgba);
        pcl::toROSMsg(cloud_rot_xyz, cloud_ros_xyz);

        cloud_ros_xyzrgba.header.frame_id = "imu_pose";
        cloud_ros_xyzrgba.header.seq = count;
        cloud_ros_xyzrgba.header.stamp = cloud_ros->header.stamp;

        cloud_ros_xyz.header.frame_id = "imu_pose";
        cloud_ros_xyz.header.seq = count;
        cloud_ros_xyz.header.stamp = cloud_ros->header.stamp;
        count++;

        pub_xyzrgba.publish(cloud_ros_xyzrgba);
        pub_xyz.publish(cloud_ros_xyz);


        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(
                tf::Quaternion(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z));

        stCloudRot = tf::StampedTransform(transform, cloud_ros->header.stamp, "world", "imu_pose");
        stImu = tf::StampedTransform(transform, cloud_ros->header.stamp, "world", "imu_pose");

        broadcaster.sendTransform(stCloudRot);
        broadcaster.sendTransform(stImu);
    }

    void computeRotationMatrix(const float W,
                               const float X,
                               const float Y,
                               const float Z,
                               Eigen::Matrix3f &rotation_matrix)
    {
        Eigen::Quaternionf quaternionf(W, X, Y, Z);
        Eigen::Matrix3f m1, m2, m3;

        m1 << 0, 0, 1,
                -1, 0, 0,
                0, -1, 0;

        m2 = quaternionf.normalized().toRotationMatrix();

        m3 << 0, 0, -1,
                1, 0, 0,
                0, -1, 0;

        rotation_matrix = m3 * m2 * m1;
    }

    void rotationCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in,
                             const Eigen::Matrix3f rotation_matrix,
                             pcl::PointCloud<pcl::PointXYZRGBA> &cloud_rotation,
                             pcl::PointCloud<pcl::PointXYZ> &cloud_rotation_xyz)
    {
        cloud_rotation.is_dense = true;
        cloud_rotation.resize(512 * 424);

        cloud_rotation_xyz.is_dense = true;
        cloud_rotation_xyz.resize(512 * 424);

        float r00, r01, r02, r10, r11, r12, r20, r21, r22;

        r00 = rotation_matrix(0, 0);
        r01 = rotation_matrix(0, 1);
        r02 = rotation_matrix(0, 2);
        r10 = rotation_matrix(1, 0);
        r11 = rotation_matrix(1, 1);
        r12 = rotation_matrix(1, 2);
        r20 = rotation_matrix(2, 0);
        r21 = rotation_matrix(2, 1);
        r22 = rotation_matrix(2, 2);

        size_t height = 424, width = 512;
        size_t index = 0;

        for (size_t i = 0; i < height; i = i + 1)
        {
            for (size_t j = 0; j < width; j = j + 1)
            {
                index = i * width + j;

                cloud_rotation.points[index].r = cloud_in.points[index].r;
                cloud_rotation.points[index].g = cloud_in.points[index].g;
                cloud_rotation.points[index].b = cloud_in.points[index].b;
                cloud_rotation.points[index].a = cloud_in.points[index].a;
                if ((!std::isnan(cloud_in.points[index].r))
                    && (!std::isnan(cloud_in.points[index].x)))
                {
                    cloud_rotation.points[index].x =
                            r00 * cloud_in.points[index].x +
                            r01 * cloud_in.points[index].y +
                            r02 * cloud_in.points[index].z;
                    cloud_rotation.points[index].y =
                            r10 * cloud_in.points[index].x +
                            r11 * cloud_in.points[index].y +
                            r12 * cloud_in.points[index].z;
                    cloud_rotation.points[index].z =
                            r20 * cloud_in.points[index].x +
                            r21 * cloud_in.points[index].y +
                            r22 * cloud_in.points[index].z;

                } else
                {
                    cloud_rotation.points[index].x = std::numeric_limits<float>::quiet_NaN();
                    cloud_rotation.points[index].y = std::numeric_limits<float>::quiet_NaN();
                    cloud_rotation.points[index].z = std::numeric_limits<float>::quiet_NaN();
                }

                cloud_rotation_xyz.points[index].x = cloud_rotation.points[index].x;
                cloud_rotation_xyz.points[index].y = cloud_rotation.points[index].y;
                cloud_rotation_xyz.points[index].z = cloud_rotation.points[index].z;
            }
        }
        cloud_rotation.width = static_cast<uint32_t>(width);
        cloud_rotation.height = static_cast<uint32_t>(height);

        cloud_rotation_xyz.width = static_cast<uint32_t>(width);
        cloud_rotation_xyz.height = static_cast<uint32_t>(height);
    }

private:
    ros::NodeHandle private_nh;

    int queueSize;
    bool useExact;

    std::string topicPointCloud2, topicImu;
    std::string topicPointCloudRotXYZRGBA, topicPointCloudRotXYZ;

    ros::Publisher pub_xyzrgba;
    ros::Publisher pub_xyz;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *subPointCloud2;
    message_filters::Subscriber<sensor_msgs::Imu> *subImu;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Imu> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu> ApproximateSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    Eigen::Matrix3f rotation_matrix;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_unrot, cloud_rot;
    pcl::PointCloud<pcl::PointXYZ> cloud_rot_xyz;
    sensor_msgs::PointCloud2 cloud_ros_xyzrgba, cloud_ros_xyz;

    unsigned long count;

    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform stCloudRot, stImu;


};

class CloudRotationNodelet : public nodelet::Nodelet
{
public:
    CloudRotationNodelet() : Nodelet(), pcloud_rotation(nullptr) {}

    ~CloudRotationNodelet() override { delete pcloud_rotation; }

private:
    void onInit() override
    {
        pcloud_rotation = new CloudRotation(getPrivateNodeHandle());
        pcloud_rotation->init();
    }

    CloudRotation *pcloud_rotation;
};

PLUGINLIB_DECLARE_CLASS(CloudRotation, CloudRotationNodelet, CloudRotationNodelet, nodelet::Nodelet);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_rotation");
    ros::NodeHandle private_nh("~");

    CloudRotation cloudRotation(private_nh,"buxx/sd/points","buxx/imu_reader/imu_data");

    cloudRotation.init();


    ros::spin();

    ros::shutdown();
    return 0;
}
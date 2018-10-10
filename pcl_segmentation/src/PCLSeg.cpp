//
// Created by zxm on 18-8-18.
//

#include "pcl_segmentation/PCLSeg.h"

#include <ros/ros.h>

#include <chrono>
using namespace std::chrono;

#define SHOWTIME

double tic_toc_ms()
{
    static auto last = system_clock::now();
    auto duration = duration_cast<microseconds>(system_clock::now() - last);
    last = system_clock::now();
    return duration.count() / 1000.0;
}

void PCLSeg::process(CloudType &cloud,std::vector<CloudType> &vec_planes, std::vector<double> &vectime)
{
    clear();

    double t;
#ifdef SHOWTIME
    ROS_INFO("***************** pcl_seg process in ***************** ");
    tic_toc_ms();
#endif

//    remove_nan_points(cloud);
//
//#ifdef SHOWTIME
//    ROS_INFO("remove_nan_points time: %.2f ms", tic_toc_ms());
//#endif

    normalEstimationIntegral(cloud, normal);

    t = tic_toc_ms();
    vectime.push_back(t);
#ifdef SHOWTIME
    ROS_INFO("normalEstimationIntegral time: %.2f ms", t);
#endif

    passThroughCloudANDNormal(cloud, normal, cloud_ps, normal_ps);

    t = tic_toc_ms();
    vectime.push_back(t);
#ifdef SHOWTIME
    ROS_INFO("passThroughCloudANDNormal time: %.2f ms", tic_toc_ms());
#endif

    if (cloud_ps.points.size() < 100)
    {
        return;
    }

    voxelGridCloudANDNormal(cloud_ps, normal_ps, voxel_x, voxel_y, voxel_z, cloud_down_sample, normal_down_sample);

    t = tic_toc_ms();
    vectime.push_back(t);
#ifdef SHOWTIME
    ROS_INFO("voxelGridCloudANDNormal time: %.2f ms", t);
#endif

    removeNANNormalPoints(cloud_down_sample, normal_down_sample, cloud_ps_nonan, normal_ps_nonan);

//    t = tic_toc_ms();
//    vectime.push_back(t);
//#ifdef SHOWTIME
//    ROS_INFO("removeNANNormalPoints time: %.2f ms", t);
//#endif

    removeClosetPoints(cloud_ps_nonan, normal_ps_nonan, cloud_no_leg, normal_no_leg, noleg_distance);

#ifdef SHOWTIME
    ROS_INFO("removeClosetPoints time: %.2f ms", t);
#endif

    extractPerpendicularPlanePoints(cloud_no_leg, normal_no_leg, Eigen::Vector3f(1, 0, 0),
                                    pcl::deg2rad(parallel_angle_diff), pcl::deg2rad(perpendicular_angle_diff),
                                    cloud_h, normal_h, cloud_v, normal_v);

    t = tic_toc_ms();
    vectime.push_back(t);
#ifdef SHOWTIME
    ROS_INFO("extractPerpendicularPlanePoints time: %.2f ms", t);
#endif

    segmentHorizontalPlanes(cloud_h, normal_h, seg_max_iters,
                            seg_threshold, pcl::deg2rad(seg_plane_angle_diff),
                            seg_rest_point, vector_cloud_h, vector_coefficients_h);

    t = tic_toc_ms();
    vectime.push_back(t);
#ifdef SHOWTIME
    ROS_INFO("segmentHorizontalPlanes time: %.2f ms", t);
#endif

    segmentVerticalPlanes(cloud_v, seg_max_iters, seg_threshold,
                          pcl::deg2rad(seg_plane_angle_diff), seg_rest_point,
                          vector_cloud_v, vector_coefficients_v);

    t = tic_toc_ms();
    vectime.push_back(t);
#ifdef SHOWTIME
    ROS_INFO("segmentVerticalPlanes time: %.2f ms", t);
#endif

    vec_planes.insert(vec_planes.begin(),vector_cloud_h.begin(),vector_cloud_h.end());
    vec_planes.insert(vec_planes.begin(),vector_cloud_v.begin(),vector_cloud_v.end());

#ifdef SHOWTIME
    ROS_INFO("***************** pcl_seg process out ***************** ");
#endif
}

void PCLSeg::clear()
{
    vector_cloud_h.clear();
    vector_coefficients_h.clear();
    vector_cloud_v.clear();
    vector_coefficients_v.clear();
}

void PCLSeg::remove_nan_points(CloudType &cloud)
{
    size_t height = cloud.height;
    size_t width = cloud.width;
    for (size_t i = 0; i < height; i++)
    {
        for (size_t j = 0; j < width; j++)
        {
            if (cloud.points[i * width + j].r == 0 &&
                cloud.points[i * width + j].g == 0 &&
                cloud.points[i * width + j].b == 0)
            {
                cloud.points[i * width + j].x = std::numeric_limits<float>::quiet_NaN();
                cloud.points[i * width + j].y = std::numeric_limits<float>::quiet_NaN();
                cloud.points[i * width + j].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
}

/** \brief Normal estimation using method in paper
    * "Real-Time Plane Segmentation using RGB-D Cameras " by Dirk.
    * This method takes advantages of organized image and is
    * much faster than normal normal estimation method based on
    * unorganized point cloud
    */
void PCLSeg::normalEstimationIntegral(const pcl::PointCloud<PointType> &cloud_in,
                                      pcl::PointCloud<pcl::Normal> &normal_out)
{
    pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
    ne.setNormalEstimationMethod(normalEstimationMethod);
    ne.setMaxDepthChangeFactor(depth_change_factor);
    ne.setNormalSmoothingSize(normal_smoothing_size);
    ne.setInputCloud(cloud_in.makeShared());
    ne.compute(normal_out);
}

/** \brief Benefited from normals have already estimated, this pass
    * through both points cloud and normals based on there coordinate
    */
void PCLSeg::passThroughCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
                                       const pcl::PointCloud<pcl::Normal> &normal_in,
                                       pcl::PointCloud<PointType> &cloud_out,
                                       pcl::PointCloud<pcl::Normal> &normal_out)
{
    pcl::PointCloud<PointType>::Ptr tmp_cloud_ptr(
            new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr tmp_normal_ptr(
            new pcl::PointCloud<pcl::Normal>);

    pcl::ExtractIndices<PointType> extract_points;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    // passthrough filter x
    indices->indices.clear();
    pcl::PassThrough<PointType> pass_through;
    pass_through.setInputCloud(cloud_in.makeShared());
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(x_min, x_max);
    pass_through.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(cloud_in.makeShared());
    extract_points.setNegative(false);
    extract_points.filter(*tmp_cloud_ptr);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(normal_in.makeShared());
    extract_normals.setNegative(false);
    extract_normals.filter(*tmp_normal_ptr);

    // passthrough filter y
    indices->indices.clear();
    pass_through.setInputCloud(tmp_cloud_ptr);
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(y_min, y_max);
    pass_through.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(tmp_cloud_ptr);
    extract_points.setNegative(false);
    extract_points.filter(*tmp_cloud_ptr);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(tmp_normal_ptr);
    extract_normals.setNegative(false);
    extract_normals.filter(*tmp_normal_ptr);

    // passthrough filter z
    indices->indices.clear();
    pass_through.setInputCloud(tmp_cloud_ptr);
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(z_min, z_max);
    pass_through.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(tmp_cloud_ptr);
    extract_points.setNegative(false);
    extract_points.filter(cloud_out);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(tmp_normal_ptr);
    extract_normals.setNegative(false);
    extract_normals.filter(normal_out);

    cloud_out.is_dense = false;
}


/** \brief A modified VoxelGrid who returns indices of points
    * in input cloud which closet to there centriod
    */
void PCLSeg::voxelGridCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
                                     const pcl::PointCloud<pcl::Normal> &normal_in,
                                     float leaf_size_x, float leaf_size_y, float leaf_size_z,
                                     pcl::PointCloud<PointType> &cloud_out,
                                     pcl::PointCloud<pcl::Normal> &normal_out)
{
    pcl::ExtractIndices<PointType> extract_points;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    pcl::VoxelGridIndices<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud_in.makeShared());
    voxel_grid.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    voxel_grid.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(cloud_in.makeShared());
    extract_points.setNegative(false);
    extract_points.filter(cloud_out);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(normal_in.makeShared());
    extract_normals.setNegative(false);
    extract_normals.filter(normal_out);
}

/** \brief After pass through and voxel grid, all the points is invalid,
    * but there still has some normals still invalid because of normals are
    * computed using "IntegralImageNormalEstimation"
    */
void PCLSeg::removeNANNormalPoints(const pcl::PointCloud<PointType> &cloud_in,
                                   const pcl::PointCloud<pcl::Normal> &normal_in,
                                   pcl::PointCloud<PointType> &cloud_out,
                                   pcl::PointCloud<pcl::Normal> &normal_out)
{
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ExtractIndices<PointType> extract_points;

    removeNaNNormalsFromPointCloud(normal_in, normal_out, indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(cloud_in.makeShared());
    extract_points.setNegative(false);
    extract_points.filter(cloud_out);
}


// remove points too close
void PCLSeg::removeClosetPoints(const CloudType &cloud_in, const pcl::PointCloud<pcl::Normal> &normal_in,
                                CloudType &cloud_out, pcl::PointCloud<pcl::Normal> &normal_out, float th)
{
    PointType point;
    pcl::Normal normal;
    cloud_out.points.clear();
    normal_out.points.clear();
    for (size_t i = 0; i < cloud_in.points.size(); i++)
    {
        point = cloud_in.points[i];
        if (point.x * point.x + point.y * point.y + point.y * point.y > th * th)
        {
            normal = normal_in.points[i];
            cloud_out.points.push_back(point);
            normal_out.points.push_back(normal);
        }
    }

    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    normal_out.width = normal_out.points.size();
    normal_out.height = 1;
}

/** \brief extract cloud points and normals in parallel plane which perpendicular to specific axis
    * \param[in] cloud_in: input point cloud
    * \param[in] normal_in: input normal cloud
    * \param[in] ax: axis planes perpendicular to
    * \param[in] angle_threshold: angle(rad) threshold point normal with ax
    * \param[out] cloud_parallel: output points cloud
    * \param[out] normal_parallel: output normal cloud
    */
void PCLSeg::extractPerpendicularPlanePoints(const pcl::PointCloud<PointType> &cloud_in,
                                             const pcl::PointCloud<pcl::Normal> &normal_in,
                                             const Eigen::Vector3f &ax,
                                             const double angle_threshold_1,
                                             const double angle_threshold_2,
                                             pcl::PointCloud<PointType> &cloud_parallel,
                                             pcl::PointCloud<pcl::Normal> &normal_parallel,
                                             pcl::PointCloud<PointType> &cloud_perpendicular,
                                             pcl::PointCloud<pcl::Normal> &normal_perpendicular)
{
    Eigen::Vector3f point_normal;
    double angle;

    cloud_parallel.points.clear();
    normal_parallel.points.clear();
    cloud_perpendicular.points.clear();
    normal_perpendicular.points.clear();

    for (size_t i = 0; i < normal_in.points.size(); i++)
    {
        point_normal = Eigen::Vector3f(normal_in.points[i].normal_x,
                                       normal_in.points[i].normal_y, normal_in.points[i].normal_z);

        angle = acos(point_normal.dot(ax));

        if ((angle < angle_threshold_1) || (M_PI - angle < angle_threshold_1))
        {
            cloud_parallel.points.push_back(cloud_in.points[i]);
            normal_parallel.points.push_back(normal_in.points[i]);
        } else if (fabs(angle - M_PI / 2) < angle_threshold_2)
        {
            cloud_perpendicular.points.push_back(cloud_in.points[i]);
            normal_perpendicular.points.push_back(normal_in.points[i]);
        }
    }
    cloud_parallel.width = cloud_parallel.points.size();
    cloud_parallel.height = 1;
    normal_parallel.width = normal_parallel.points.size();
    normal_parallel.height = 1;
    cloud_perpendicular.width = cloud_perpendicular.points.size();
    cloud_perpendicular.height = 1;
    normal_perpendicular.width = normal_perpendicular.points.size();
    normal_perpendicular.height = 1;
}

/** \brief segment cloud points plane which perpendicular to specific axis
    * \param[in] cloud_in: input point cloud
    * \param[in] normal_in: input normal cloud
    * \param[out] vector_cloud: output vector of points cloud
    * \param[out] vector_coefficients: output vector of plane coefficients
    */
void PCLSeg::segmentHorizontalPlanes(const pcl::PointCloud<PointType> &cloud_in,
                                     const pcl::PointCloud<pcl::Normal> &normal_in,
                                     const int seg_max_iters,
                                     const float seg_threshold,
                                     const float angle_threshold,
                                     const int min_plane_points,
                                     std::vector<pcl::PointCloud<PointType> > &vector_cloud,
                                     std::vector<pcl::ModelCoefficients> &vector_coefficients)
{
    pcl::PointCloud<PointType> rest_points = cloud_in;
//      pcl::PointCloud<pcl::Normal> rest_normals = normal_in;

    if (rest_points.points.size() == 0)
        return;

    vector_cloud.clear();
    vector_coefficients.clear();

    std::vector<pcl::ModelCoefficients> vector_plane_coefficients;
    std::vector<pcl::PointIndices> vector_plane_inliers;

    // Create the segmentation object
    pcl::SACSegmentation<PointType> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(seg_max_iters);
    seg.setDistanceThreshold(seg_threshold);
    seg.setAxis(Eigen::Vector3f(1, 0, 0));
    seg.setEpsAngle(angle_threshold);

    // extract indices object
    pcl::ExtractIndices<PointType> extract_points;

    std::vector<pcl::PointCloud<PointType> > vector_cluster;
    pointCluster(rest_points, vector_cluster);
    for (int i = 0; i < vector_cluster.size(); i++)
    {
        pcl::PointCloud<PointType> iter_points = vector_cluster[i];

        for (;;)
        {
            if (iter_points.points.size() < min_plane_points)
                break;

            pcl::PointCloud<PointType> plane_inliers_points;
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients plane_coefficients;
            seg.setInputCloud(iter_points.makeShared());
            seg.segment(*plane_inliers, plane_coefficients);

            if (plane_inliers->indices.size() < min_plane_points)
                break;

            extract_points.setInputCloud(iter_points.makeShared());
            extract_points.setIndices(plane_inliers);
            extract_points.setNegative(false);
            extract_points.filter(plane_inliers_points);
            extract_points.setNegative(true);
            extract_points.filter(iter_points);

            vector_cloud.push_back(plane_inliers_points);
            vector_coefficients.push_back(plane_coefficients);

            if (iter_points.points.size() < min_plane_points)
                break;
        }
    }
}

/** \brief segment cloud points plane which perpendicular to specific axis
    * \param[in] cloud_in: input point cloud
    * \param[in] normal_in: input normal cloud
    * \param[out] vector_cloud: output vector of points cloud
    * \param[out] vector_coefficients: output vector of plane coefficients
    */
void PCLSeg::segmentVerticalPlanes(const pcl::PointCloud<PointType> &cloud_in,
                                   const int seg_max_iters,
                                   const float seg_threshold,
                                   const float angle_threshold,
                                   const int min_plane_points,
                                   std::vector<pcl::PointCloud<PointType> > &vector_cloud,
                                   std::vector<pcl::ModelCoefficients> &vector_coefficients)
{
    pcl::PointCloud<PointType> rest_points = cloud_in;

    if (rest_points.points.size() == 0)
        return;

    std::vector<pcl::ModelCoefficients> vector_plane_coefficients;
    std::vector<pcl::PointIndices> vector_plane_inliers;
    // segmentation object
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f(1, 0, 0));
    seg.setEpsAngle(angle_threshold);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(seg_max_iters);
    seg.setDistanceThreshold(seg_threshold);

    // extract indices object
    pcl::ExtractIndices<PointType> extract_points;

    std::vector<pcl::PointCloud<PointType> > vector_cluster;
    pointCluster(rest_points, vector_cluster);
    for (int i = 0; i < vector_cluster.size(); i++)
    {
        pcl::PointCloud<PointType> iter_points = vector_cluster[i];


        for (;;)
        {
            pcl::PointCloud<PointType> plane_inliers_points;
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients plane_coefficients;
            seg.setInputCloud(iter_points.makeShared());
            seg.segment(*plane_inliers, plane_coefficients);

            if (plane_inliers->indices.size() < min_plane_points)
                break;

            extract_points.setInputCloud(iter_points.makeShared());
            extract_points.setIndices(plane_inliers);
            extract_points.setNegative(false);
            extract_points.filter(plane_inliers_points);
            extract_points.setNegative(true);
            extract_points.filter(iter_points);

            vector_cloud.push_back(plane_inliers_points);
            vector_coefficients.push_back(plane_coefficients);

            if (iter_points.points.size() < min_plane_points)
                break;
        }
    }
}

void PCLSeg::pointCluster(const pcl::PointCloud<PointType> &cloud_in,
                          std::vector<pcl::PointCloud<PointType> > &vec_cluster)
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud_in.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cluster_tolerance); // 7cm
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(9999999);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in.makeShared());
    ec.extract(cluster_indices);

    vec_cluster.clear();

    pcl::ExtractIndices<PointType> extract_points;
    for (int i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<PointType> point_cloud;
        extract_points.setInputCloud(cloud_in.makeShared());
        extract_points.setNegative(false);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);
        indices->indices = cluster_indices[i].indices;
        extract_points.setIndices(indices);
        extract_points.filter(point_cloud);

        vec_cluster.push_back(point_cloud);
    }
}


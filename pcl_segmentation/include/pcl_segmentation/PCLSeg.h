//
// Created by zxm on 18-8-18.
//

#ifndef PROJECT_PCL_SEG_H
#define PROJECT_PCL_SEG_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <omp.h>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/common/time.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_segmentation/voxel_grid_indices.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> CloudType;

class PCLSeg
{
public:
    PCLSeg():
    x_min(-5),x_max(5),y_min(-5),y_max(5),z_min(-5),z_max(5),
    normalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::NormalEstimationMethod::AVERAGE_3D_GRADIENT),
    depth_change_factor(0.02),normal_smoothing_size(5.0),
    voxel_x(0.01),voxel_y(0.01),voxel_z(0.01),
    parallel_angle_diff(19),perpendicular_angle_diff(24),
    noleg_distance(0.6),seg_threshold(0.01),seg_plane_angle_diff(7.0),
    seg_rest_point(230),seg_max_iters(100),cluster_tolerance(0.03),min_cluster_size(9)
    {}

    void clear();

    void process(CloudType &cloud,std::vector<CloudType> &vec_planes, std::vector<double> &vectime);

    void remove_nan_points(CloudType &cloud);

    void normalEstimationIntegral(const pcl::PointCloud<PointType> &cloud_in,
                                  pcl::PointCloud<pcl::Normal> &normal_out);

    void passThroughCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
                                   const pcl::PointCloud<pcl::Normal> &normal_in,
                                   pcl::PointCloud<PointType> &cloud_out,
                                   pcl::PointCloud<pcl::Normal> &normal_out);

    void voxelGridCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
                                 const pcl::PointCloud<pcl::Normal> &normal_in,
                                 float leaf_size_x, float leaf_size_y, float leaf_size_z,
                                 pcl::PointCloud<PointType> &cloud_out,
                                 pcl::PointCloud<pcl::Normal> &normal_out);

    void removeNANNormalPoints(const pcl::PointCloud<PointType> &cloud_in,
                               const pcl::PointCloud<pcl::Normal> &normal_in,
                               pcl::PointCloud<PointType> &cloud_out,
                               pcl::PointCloud<pcl::Normal> &normal_out);

    void removeClosetPoints(const CloudType &cloud_in, const pcl::PointCloud<pcl::Normal> &normal_in,
                            CloudType &cloud_out, pcl::PointCloud<pcl::Normal> &normal_out, float th);

    void extractPerpendicularPlanePoints(const pcl::PointCloud<PointType> &cloud_in,
                                         const pcl::PointCloud<pcl::Normal> &normal_in,
                                         const Eigen::Vector3f &ax,
                                         const double angle_threshold_1,
                                         const double angle_threshold_2,
                                         pcl::PointCloud<PointType> &cloud_parallel,
                                         pcl::PointCloud<pcl::Normal> &normal_parallel,
                                         pcl::PointCloud<PointType> &cloud_perpendicular,
                                         pcl::PointCloud<pcl::Normal> &normal_perpendicular);

    void segmentHorizontalPlanes(const pcl::PointCloud<PointType> &cloud_in,
                                 const pcl::PointCloud<pcl::Normal> &normal_in,
                                 const int seg_max_iters,
                                 const float seg_threshold,
                                 const float angle_threshold,
                                 const int min_plane_points,
                                 std::vector<pcl::PointCloud<PointType> > &vector_cloud,
                                 std::vector<pcl::ModelCoefficients> &vector_coefficients);

    void segmentVerticalPlanes(const pcl::PointCloud<PointType> &cloud_in,
                               const int seg_max_iters,
                               const float seg_threshold,
                               const float angle_threshold,
                               const int min_plane_points,
                               std::vector<pcl::PointCloud<PointType> > &vector_cloud,
                               std::vector<pcl::ModelCoefficients> &vector_coefficients);

    void pointCluster(const pcl::PointCloud<PointType> &cloud_in,
                      std::vector<pcl::PointCloud<PointType> > &vec_cluster);

    // preprocess parameters
    // range limit of point cloud after rotation
    float x_min;// = 0.55;
    float x_max;// = 1.5;
    float y_min;// = 0;
    float y_max;// = 1;
    float z_min;// = -0.5;
    float z_max;// = 0.5;


    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::NormalEstimationMethod normalEstimationMethod;// AVERAGE_3D_GRADIENT
    float depth_change_factor;// 0.02;
    float normal_smoothing_size;//5.0

    // voxel gird
    float voxel_x, voxel_y, voxel_z;//0.01,0.01,0.01

    // extract perpendicular plane
    float parallel_angle_diff;//19
    float perpendicular_angle_diff;//24

    // remove the points too close to origin
    double noleg_distance;//0.6

    // plane segmentation
    float seg_threshold, seg_plane_angle_diff;//0.01,7.0
    int seg_rest_point;//236
    // segmentation max iteration numbers
    int seg_max_iters;// 100;
    // points with distance to plane smaller than this parameter (units:m)
    // will be segment into this plane

    // find main cluster
    float cluster_tolerance; // 0.03
    int min_cluster_size; //9


private:

    pcl::PointCloud<PointType> cloud_ps, cloud_down_sample, cloud_ps_nonan, cloud_no_leg;
    pcl::PointCloud<pcl::Normal> normal, normal_ps, normal_down_sample, normal_ps_nonan, normal_no_leg;

    pcl::PointCloud<PointType> cloud_h, cloud_v;
    pcl::PointCloud<pcl::Normal> normal_h, normal_v;

    std::vector<pcl::PointCloud<PointType> > vector_cloud_h;
    std::vector<pcl::ModelCoefficients> vector_coefficients_h;
    std::vector<pcl::PointCloud<PointType> > vector_cloud_v;
    std::vector<pcl::ModelCoefficients> vector_coefficients_v;
};


#endif //PROJECT_PCL_SEG_H

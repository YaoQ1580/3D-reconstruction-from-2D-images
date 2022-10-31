#ifndef __PROPOINTCLOUD_H
#define __PROPOINTCLOUD_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/console/parse.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>


void savepntsPCD(cv::Mat& pnts);
void filterpointcloud(void);
void poissonreconstruction(void);
void viewer_ply(void);






#endif
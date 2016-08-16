/*
//類別名稱：無
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：將所有論文中用到的Point Cloud Processing都集合在這邊
//使用函式庫：PCL
*/
#pragma once
#undef max
#undef min
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <vector>
#include <string>
#include <sstream>

// pass through filter
pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& direction, const std::vector<double>& limits);
// down sampling filter
pcl::PointCloud<pcl::PointXYZ>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<double>& leafSize);
// RANSAC找平面 也會將平面參數存入coefficients
bool searchPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, int maxIter = 1000, double distanceThreshold = 0.01);
// 刪除平面跟平面以下雜點
pcl::PointCloud<pcl::PointXYZ>::Ptr stayObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);
// segmentation但沒有另外存point clouds, 只有對應原圖的indices
void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices, double tolerance = 0.05, int minSize = 50, int maxSize = 25000);
// segmentation 並且有另外存分開的 point cloud
void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& objects, double tolerance = 0.05, int minSize = 50, int maxSize = 25000);
// upSampling 這function不好用
pcl::PointCloud<pcl::PointXYZ>::Ptr upSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double searchRadius = 0.01);
// 估測normal的function 之後沒用這個
pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double searchRadius = 0.03);
// 估測normal的function 之後建出來的mesh表面很平滑
pcl::PointCloud<pcl::PointNormal>::Ptr smoothingNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double searchRadius = 0.03);
// 論文上提到的reconstruction方式
pcl::PolygonMesh::Ptr createMesh(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double searchRadius = 0.025);
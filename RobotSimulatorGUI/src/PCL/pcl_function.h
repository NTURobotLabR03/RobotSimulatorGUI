/*
//���O�W�١G�L
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�N�Ҧ��פ夤�Ψ쪺Point Cloud Processing�����X�b�o��
//�ϥΨ禡�w�GPCL
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
// RANSAC�䥭�� �]�|�N�����ѼƦs�Jcoefficients
bool searchPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, int maxIter = 1000, double distanceThreshold = 0.01);
// �R�������򥭭��H�U���I
pcl::PointCloud<pcl::PointXYZ>::Ptr stayObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);
// segmentation���S���t�~�spoint clouds, �u��������Ϫ�indices
void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices, double tolerance = 0.05, int minSize = 50, int maxSize = 25000);
// segmentation �åB���t�~�s���}�� point cloud
void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& objects, double tolerance = 0.05, int minSize = 50, int maxSize = 25000);
// upSampling �ofunction���n��
pcl::PointCloud<pcl::PointXYZ>::Ptr upSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double searchRadius = 0.01);
// ����normal��function ����S�γo��
pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double searchRadius = 0.03);
// ����normal��function ����إX�Ӫ�mesh���ܥ���
pcl::PointCloud<pcl::PointNormal>::Ptr smoothingNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double searchRadius = 0.03);
// �פ�W���쪺reconstruction�覡
pcl::PolygonMesh::Ptr createMesh(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double searchRadius = 0.025);
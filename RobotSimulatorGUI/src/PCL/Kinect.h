/*
//���O�W�١GKinect
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G��פ�һ�Kinect��T�H�ι�����Transformation Matrix�s�_��
//�ϥΨ禡�w�GPCL, Eigen
*/
#pragma once
#include "kinect_grabber.h"

typedef pcl::PointXYZ PointT;

class Kinect{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// index 1���ܬO�ĤG�xkinect
	Kinect(int index = 0, const Eigen::Matrix4d& transMat = Eigen::Matrix4d::Identity());
	~Kinect();
	// �Ұ�kinect
	void start();
	// ����kinect
	void stop();
	// �]�wtransformation matrix
	void setTransMat(const Eigen::Matrix4d& transMat);
	// �N�S��L��point cloud�ǥX�� (��point cloud)
	const pcl::PointCloud<PointT>::ConstPtr& getConstPtr();
	// output ��L�ഫ��point cloud
	pcl::PointCloud<PointT>::Ptr getPointCloud();

private:
	pcl::Grabber *_grabber;
	pcl::PointCloud<PointT>::ConstPtr *_cloud;
	boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> _function;
	Eigen::Matrix4d _transMat;
};
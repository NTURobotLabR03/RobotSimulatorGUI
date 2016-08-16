/*
//類別名稱：Kinect
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：把論文所需Kinect資訊以及對應的Transformation Matrix存起來
//使用函式庫：PCL, Eigen
*/
#pragma once
#include "kinect_grabber.h"

typedef pcl::PointXYZ PointT;

class Kinect{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// index 1的話是第二台kinect
	Kinect(int index = 0, const Eigen::Matrix4d& transMat = Eigen::Matrix4d::Identity());
	~Kinect();
	// 啟動kinect
	void start();
	// 停止kinect
	void stop();
	// 設定transformation matrix
	void setTransMat(const Eigen::Matrix4d& transMat);
	// 將沒轉過的point cloud傳出來 (原point cloud)
	const pcl::PointCloud<PointT>::ConstPtr& getConstPtr();
	// output 轉過轉換的point cloud
	pcl::PointCloud<PointT>::Ptr getPointCloud();

private:
	pcl::Grabber *_grabber;
	pcl::PointCloud<PointT>::ConstPtr *_cloud;
	boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> _function;
	Eigen::Matrix4d _transMat;
};
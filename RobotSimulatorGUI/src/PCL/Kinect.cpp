#include "stdafx.h"
#include "Kinect.h"
#include <pcl/common/transforms.h>

// global variable
static pcl::PointCloud<PointT>::ConstPtr kinect0_cloud, kinect1_cloud;

void kinect0Callback(const pcl::PointCloud<PointT>::ConstPtr& cloud){
	kinect0_cloud = cloud;
}

void kinect1Callback(const pcl::PointCloud<PointT>::ConstPtr& cloud){
	kinect1_cloud = cloud;
}

Kinect::Kinect(int index, const Eigen::Matrix4d& transMat){
	_grabber = new pcl::KinectGrabber(index);
	_transMat = transMat;
	if (index == 0){
		_function = &kinect0Callback;
		_cloud = &kinect0_cloud;
	}
	else if(index == 1){
		_function = &kinect1Callback;
		_cloud = &kinect1_cloud;
	}
	else{
		throw;
	}
	_grabber->registerCallback(_function);
}

Kinect::~Kinect(){

}

void Kinect::start(){
	_grabber->start();
}

void Kinect::stop(){
	_grabber->stop();
}

const pcl::PointCloud<PointT>::ConstPtr& Kinect::getConstPtr(){
	return *_cloud;
}

pcl::PointCloud<PointT>::Ptr Kinect::getPointCloud(){
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>());
	*cloud = **_cloud;
	transformPointCloud(*cloud, *transformedCloud, _transMat);
	return transformedCloud;
}

void Kinect::setTransMat(const Eigen::Matrix4d& transMat){
	_transMat = transMat;
}
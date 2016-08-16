#include "stdafx.h"
#include "ThreadControl.h"
#include "trajPlan.h"
#include <iostream>
#include <fstream>
using namespace pcl;
using namespace tp;
using namespace std;
using namespace Eigen;

mutex gMutex;

void recordPath(UISimulator* demo,bool* breakFlag){
	vector<VectorXd> armPath, handPath;
	while (!(*breakFlag)){
		lock_guard<mutex> mLock(gMutex);
		this_thread::sleep_for(chrono::duration<int, milli>(5));
		armPath.push_back(VectorXd());
		handPath.push_back(VectorXd());
		demo->getArmJoint(armPath.back());
		armPath.back()[5] = -armPath.back()[5]; // 特殊狀況 黑金剛問題
		demo->getHandJoint(handPath.back());
	}

	fstream fout;
	fout.open("path/ArmPath.txt", ios::out);
	for (int i = 0, n = armPath.size() - 1; i < n; ++i){
		vector<VectorXd> intePath;
		interpolation(5, armPath[i], armPath[i + 1], intePath);
		for (auto& it : intePath){
			fout << it.transpose() << endl;
		}
	}
	fout.close();
	fout.open("path/HandPath.txt", ios::out);
	for (int i = 0, n = handPath.size() - 1; i < n; ++i){
		vector<VectorXd> intePath;
		interpolation(5, handPath[i], handPath[i + 1], intePath);
		for (auto& it : intePath){
			fout << it.head(3).transpose() << " " << it.tail(8).transpose() << endl;
		}
	}
	fout.close();
}

void foo(Eigen::Matrix4d& transMat, PointCloud<PointT>::Ptr cloud0, PointCloud<PointT>::Ptr cloud1){
	PointCloud<PointT>::Ptr cloud1Trans(new PointCloud<PointT>()), cloudSum(new PointCloud<PointT>());

	visualization::CloudViewer viewer("Point Cloud Viewer");
	while (!viewer.wasStopped()){
		transformPointCloud(*cloud1, *cloud1Trans, transMat);
		*cloudSum = *cloud0 + *cloud1Trans;
		viewer.showCloud(cloudSum);
	}
}

void foo1(Kinect* kinect){
	visualization::CloudViewer viewer("Point Cloud Viewer");
	while (!viewer.wasStopped()){
		if (kinect)
			viewer.showCloud(kinect->getConstPtr());
	}
}

void foo2(PointCloud<PointT>::Ptr& cloud){
	visualization::CloudViewer viewer("Point Cloud Viewer");
	while (!viewer.wasStopped()){
		viewer.showCloud(cloud);
	}
}

void foo3(vector<PointCloud<PointT>::Ptr>& objects){
	visualization::PCLVisualizer viewer("Point Cloud Viewer");
	for (int i = 0, n = objects.size(); i < n; ++i){
		ostringstream os;
		os << i;
		int section = 255 / objects.size();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(objects[i], 255 - section * i, 0 + section * i, 0 + section * i);
		viewer.addPointCloud(objects[i], colorHandler, os.str());
		os.str("");
	}
	while (!viewer.wasStopped()){
		viewer.spinOnce();
	}
}

void viewerWColor(Eigen::Matrix4d& transMat, pcl::PointCloud<PointT>::Ptr cloud0, pcl::PointCloud<PointT>::Ptr cloud1){
	PointCloud<PointT>::Ptr cloud1Trans(new PointCloud<PointT>());
	visualization::PCLVisualizer viewer("Point Cloud Viewer");
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud0, 255, 255, 255);
		viewer.addPointCloud(cloud0, colorHandler, "0");
	}
	transformPointCloud(*cloud1, *cloud1Trans, transMat);
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud1Trans, 0, 255, 0);
		viewer.addPointCloud(cloud1Trans, colorHandler, "1");
	}

	while (!viewer.wasStopped()){
		viewer.spinOnce();
	}
}
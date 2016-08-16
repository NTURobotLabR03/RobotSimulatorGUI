/*
//類別名稱：無
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：Thread 的 callback function
//      callback function一定要靜態的函式
//      mutex使用也必須用靜態的物件
//      故將所有跟Thread相關的東西擺放至這裡
//使用函式庫：PCL, Eigen, Bullet, Qhull, FreeGLUT
*/
#pragma once
#include "pcl_function.h"
#include "Kinect.h"
#include "UISimulator.h"
#include <thread>
#include <mutex>
// 全域變數
// mutex 用來管理thread跟thread之間的共用資料狀況
// 不會請看 https://kheresy.wordpress.com/2012/07/11/multi-thread-programming-in-c-thread-p2/
extern std::mutex gMutex;

// 這個沒用到 本是用來記錄robot 每幾個ms之間的狀況 並且記錄起來
void recordPath(UISimulator* demo,bool* breakFlag);
// 這個 callback function 會將兩個point cloud 合併在一個視窗裡面 show 出來 ，其中transMat代表cloud1相對cloud0的transformation matrix
void foo(Eigen::Matrix4d& transMat, pcl::PointCloud<PointT>::Ptr cloud0, pcl::PointCloud<PointT>::Ptr cloud1);
// 將某一台kinect的收到的資料show出來
void foo1(Kinect* kinect);
// 單純show出某個point cloud
void foo2(pcl::PointCloud<PointT>::Ptr& cloud);
// 將不同的point clouds用不同顏色show出來(用於segmentation 出來後的point clouds)
void foo3(std::vector<pcl::PointCloud<PointT>::Ptr>& objects);
// 跟foo一樣 但是會用不同顏色，其中有缺點是在於 transMat改變 point cloud位置不會一起變
void viewerWColor(Eigen::Matrix4d& transMat, pcl::PointCloud<PointT>::Ptr cloud0, pcl::PointCloud<PointT>::Ptr cloud1);
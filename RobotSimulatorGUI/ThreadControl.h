/*
//���O�W�١G�L
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��GThread �� callback function
//      callback function�@�w�n�R�A���禡
//      mutex�ϥΤ]�������R�A������
//      �G�N�Ҧ���Thread�������F���\��ܳo��
//�ϥΨ禡�w�GPCL, Eigen, Bullet, Qhull, FreeGLUT
*/
#pragma once
#include "pcl_function.h"
#include "Kinect.h"
#include "UISimulator.h"
#include <thread>
#include <mutex>
// �����ܼ�
// mutex �ΨӺ޲zthread��thread�������@�θ�ƪ��p
// ���|�Ь� https://kheresy.wordpress.com/2012/07/11/multi-thread-programming-in-c-thread-p2/
extern std::mutex gMutex;

// �o�ӨS�Ψ� ���O�ΨӰO��robot �C�X��ms���������p �åB�O���_��
void recordPath(UISimulator* demo,bool* breakFlag);
// �o�� callback function �|�N���point cloud �X�֦b�@�ӵ����̭� show �X�� �A�䤤transMat�N��cloud1�۹�cloud0��transformation matrix
void foo(Eigen::Matrix4d& transMat, pcl::PointCloud<PointT>::Ptr cloud0, pcl::PointCloud<PointT>::Ptr cloud1);
// �N�Y�@�xkinect�����쪺���show�X��
void foo1(Kinect* kinect);
// ���show�X�Y��point cloud
void foo2(pcl::PointCloud<PointT>::Ptr& cloud);
// �N���P��point clouds�Τ��P�C��show�X��(�Ω�segmentation �X�ӫ᪺point clouds)
void foo3(std::vector<pcl::PointCloud<PointT>::Ptr>& objects);
// ��foo�@�� ���O�|�Τ��P�C��A�䤤�����I�O�b�� transMat���� point cloud��m���|�@�_��
void viewerWColor(Eigen::Matrix4d& transMat, pcl::PointCloud<PointT>::Ptr cloud0, pcl::PointCloud<PointT>::Ptr cloud1);
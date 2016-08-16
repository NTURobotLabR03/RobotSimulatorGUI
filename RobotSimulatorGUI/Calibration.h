#pragma once
#include "Kinect.h"
#include "pcl_function.h"
#include "EigenHeader.h"
#include "ThreadControl.h"

// Calibration 對話方塊

class Calibration : public CDialogEx
{
	DECLARE_DYNAMIC(Calibration)

public:
	Calibration(Kinect* kinect0 = nullptr, Kinect* kinect1 = nullptr, Eigen::Matrix4d* transMatPtr = nullptr, CWnd* pParent = NULL);   // 標準建構函式
	virtual ~Calibration();

// 對話方塊資料
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

	DECLARE_MESSAGE_MAP()
	Kinect* _kinect0 = nullptr, *_kinect1 = nullptr;
	Eigen::Matrix4d* _transMatPtr = nullptr;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> _planeVector;
	std::thread* _thread = nullptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud0, _cloud1;
public:
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton33();
	afx_msg void OnBnClickedButton34();
	afx_msg void OnBnClickedButton35();
	afx_msg void OnBnClickedButton36();
	afx_msg void OnBnClickedButton37();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton1();
};
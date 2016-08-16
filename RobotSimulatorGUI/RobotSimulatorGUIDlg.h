/*
//類別名稱：CRobotSimulatorGUIDlg
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：MFC介面設定
//      詳細初始化OnInitDialog()去看cpp定義
//      其他buttom對應function也可以去cpp看
//使用函式庫：Eigen, Bullet, PCL, FreeGLUT, Qhull
*/
// RobotSimulatorGUIDlg.h : 標頭檔
//

#pragma once
#include "Calibration.h"
#include "UISimulator.h"
#include "HandArmState.h"
#include "PathPlayBack.h"
#include "Kinect.h"
#include "pcl_function.h"
#include "ThreadControl.h"
#include "afxwin.h"
#include <vector>

// CRobotSimulatorGUIDlg 對話方塊
class CRobotSimulatorGUIDlg : public CDialogEx
{
// 建構
public:
	CRobotSimulatorGUIDlg(CWnd* pParent = NULL);	// 標準建構函式

// 對話方塊資料
	enum { IDD = IDD_ROBOTSIMULATORGUI_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支援


// 程式碼實作
protected:
	HICON m_hIcon;

	// 產生的訊息對應函式
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
	// TODO
	UISimulator* _demo;
	std::thread _simulatorThread;
public:
	// for simulator
	afx_msg void OnBnClickedGraspplanning();
	afx_msg void OnBnClickedPathplanning();
	afx_msg void OnBnClickedPathsmoothing();
	afx_msg void OnBnClickedAabb();
	afx_msg void OnBnClickedContact();
	afx_msg void OnBnClickedRrt();
	CButton AABB_btn;
	CButton contact_btn;
	CButton rrt_btn;
	CButton path_btn;
	afx_msg void OnBnClickedPath();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnEnChangeEdit1();
	Eigen::Vector3d _tablePosition;
	afx_msg void OnDeltaposSpin1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnEnChangeEdit3();
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnDeltaposSpin3(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnDeltaposSpin2(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedButton2();
	std::vector<Object*> _objects;
	afx_msg void OnBnClickedButton3();
	CComboBox m_cbObject;
	afx_msg void OnCbnSelchangeCombo2();
	afx_msg void OnEnChangeObjectx();
	afx_msg void OnEnChangeObjecty();
	afx_msg void OnEnChangeObjectz();
	afx_msg void OnEnChangeObjectrx();
	afx_msg void OnEnChangeObjectry();
	afx_msg void OnEnChangeObjectrz();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton6();

	std::vector<double> _armJointValues;
	afx_msg void OnEnChangeArm0();
	afx_msg void OnEnChangeArm1();
	afx_msg void OnEnChangeArm2();
	afx_msg void OnEnChangeArm3();
	afx_msg void OnEnChangeArm4();
	afx_msg void OnEnChangeArm5();
	afx_msg void OnBnClickedQinit();
	afx_msg void OnBnClickedQgoal();

	HandArmState _Qinit, _Qgoal;
	bool _isGrasp = false;
	afx_msg void OnBnClickedButton8();
	afx_msg void OnBnClickedButton9();

	std::vector<double> _placeJointValues;
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedButton10();
	CButton m_IKenable;
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnEnChangeIkx();
	afx_msg void OnEnChangeIky();
	afx_msg void OnEnChangeIkz();
	afx_msg void OnEnChangeIkrx();
	afx_msg void OnEnChangeIkry();
	afx_msg void OnEnChangeIkrz();
	afx_msg void OnBnClickedButton12();

	Eigen::Matrix4d _targetPosMat;
	std::vector<PathPlayBack*> _paths;
	afx_msg void OnBnClickedButton13();
	afx_msg void OnBnClickedButton14();

	// for PCL
	Eigen::Matrix4d _transMat, _T2base;
	std::thread* _thread = 0;
	Kinect* _kinect0 = 0, *_kinect1 = 0;
	pcl::PointCloud<PointT>::Ptr _cloud;
	std::vector<pcl::PointCloud<PointT>::Ptr> _pclObjects;
	std::vector<double> _xlimits, _ylimits, _zlimits;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> _normals; // with points
	std::vector<pcl::PolygonMesh::Ptr> _meshes;
	std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > _centroid;
	afx_msg void OnBnClickedButton15();
	afx_msg void OnBnClickedButton16();
	afx_msg void OnBnClickedButton17();
	afx_msg void OnBnClickedButton18();
	afx_msg void OnBnClickedButton19();
	afx_msg void OnBnClickedButton20();
	afx_msg void OnBnClickedButton21();
	afx_msg void OnBnClickedButton22();
	afx_msg void OnBnClickedButton23();
	afx_msg void OnBnClickedButton24();
	afx_msg void OnBnClickedButton25();
	afx_msg void OnBnClickedButton26();
	afx_msg void OnBnClickedButton27();
	afx_msg void OnBnClickedButton30();
	afx_msg void OnEnChangeEdit4();
	afx_msg void OnEnChangeEdit5();
	afx_msg void OnEnChangeEdit6();
	afx_msg void OnEnChangeEdit7();
	afx_msg void OnEnChangeEdit8();
	afx_msg void OnEnChangeEdit9();
	CComboBox m_cbTransMat;
	afx_msg void OnCbnSelchangeCombo3();
	afx_msg void OnEnChangeTransx();
	afx_msg void OnEnChangeTransy();
	afx_msg void OnEnChangeTransz();
	afx_msg void OnEnChangeTransrx();
	afx_msg void OnEnChangeTransry();
	afx_msg void OnEnChangeTransrz();
	afx_msg void OnBnClickedButton28();
	afx_msg void OnBnClickedButton29();
	afx_msg void OnBnClickedButton31();
	afx_msg void OnBnClickedButton32();

	Calibration *_pDlg = nullptr;
};

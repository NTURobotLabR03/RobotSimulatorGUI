// Calibration.cpp : 實作檔
//

#include "stdafx.h"
#include "RobotSimulatorGUI.h"
#include "Calibration.h"
#include "afxdialogex.h"
using namespace std;
using namespace Eigen;
using namespace pcl;

// Calibration 對話方塊

IMPLEMENT_DYNAMIC(Calibration, CDialogEx)

Calibration::Calibration(Kinect* kinect0, Kinect* kinect1, Eigen::Matrix4d* transMatPtr, CWnd* pParent)
	: CDialogEx(Calibration::IDD, pParent)
{
	_kinect0 = kinect0;
	_kinect1 = kinect1;
	_transMatPtr = transMatPtr;
}

Calibration::~Calibration()
{
}

void Calibration::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(Calibration, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON2, &Calibration::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &Calibration::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &Calibration::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON33, &Calibration::OnBnClickedButton33)
	ON_BN_CLICKED(IDC_BUTTON34, &Calibration::OnBnClickedButton34)
	ON_BN_CLICKED(IDC_BUTTON35, &Calibration::OnBnClickedButton35)
	ON_BN_CLICKED(IDC_BUTTON36, &Calibration::OnBnClickedButton36)
	ON_BN_CLICKED(IDC_BUTTON37, &Calibration::OnBnClickedButton37)
	ON_BN_CLICKED(IDC_BUTTON5, &Calibration::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON1, &Calibration::OnBnClickedButton1)
END_MESSAGE_MAP()


// Calibration 訊息處理常式


void Calibration::OnBnClickedButton2()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	PointCloud<PointXYZ>::Ptr cloud0(new PointCloud<PointXYZ>), cloud1(new PointCloud<PointXYZ>), cloudTrans(new PointCloud<PointXYZ>);
	//過濾副檔名
	CString szFilter = CString("Polygon File Format|*.ply|All File|*.*||");
	//以Dialog方式開啟檔案
	{
		CFileDialog fd(true, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
		if (fd.DoModal() == IDOK)  //按下OK 
		{
			//開啟檔案成功
			CString szFileName = fd.GetPathName(); //取得開啟檔案的全名(包含路徑)
			io::loadPLYFile(string(CT2A(szFileName)), *cloud0);
		}
		else{
			return;
		}
	}
	//以Dialog方式開啟檔案
	{
		CFileDialog fd(true, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
		if (fd.DoModal() == IDOK)  //按下OK 
		{
			//開啟檔案成功
			CString szFileName = fd.GetPathName(); //取得開啟檔案的全名(包含路徑)
			io::loadPLYFile(string(CT2A(szFileName)), *cloud1);
		}
		else{
			return;
		}
	}

	PointIndices::Ptr inliers0(new PointIndices), inliers1(new PointIndices);
	ModelCoefficients::Ptr coefficients0(new ModelCoefficients), coefficients1(new ModelCoefficients);
	searchPlane(cloud0, inliers0, coefficients0);
	searchPlane(cloud1, inliers1, coefficients1);
	Vector3d n0, n1;
	n0 << coefficients0->values[0], coefficients0->values[1], coefficients0->values[2];
	n1 << coefficients1->values[0], coefficients1->values[1], coefficients1->values[2];
	Matrix3d R;
	a2bRotation(n1, n0, R);

	vector<Vector3d, aligned_allocator<Vector3d>> t0;
	findPlaneVectors(n0, t0);

	Matrix4d& T = *_transMatPtr;
	T = Matrix4d::Identity();
	T.block<3, 3>(0, 0) = R;
	transformPointCloud(*cloud1, *cloudTrans, T);
	searchPlane(cloudTrans, inliers1, coefficients1);

	double dis = coefficients1->values[3] - coefficients0->values[3];
	Vector3d Dis = n0*dis;
	T.block<3, 1>(0, 3) = Dis;
	//transformPointCloud(*cloud1, *cloudTrans, T);
	//searchPlane(cloudTrans, inliers1, coefficients1);

	_planeVector = t0;
	_planeVector.push_back(n0);
	_cloud0 = cloud0;
	_cloud1 = cloud1;
}


void Calibration::OnBnClickedButton3()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_thread){
		_thread->join();
		delete _thread;
	}
	_thread = new thread([=]{foo(*_transMatPtr, _cloud0, _cloud1); });
}


void Calibration::OnBnClickedButton4()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemText(IDC_EDIT1, cstring);
	try{
		double dis = stod(string(CT2A(cstring)));
		_transMatPtr->block<3, 1>(0, 3) = _transMatPtr->block<3, 1>(0, 3).eval() + dis*_planeVector[0];
	}
	catch (...){

	}
}


void Calibration::OnBnClickedButton33()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemText(IDC_EDIT1, cstring);
	try{
		double dis = stod(string(CT2A(cstring)));
		_transMatPtr->block<3, 1>(0, 3) = _transMatPtr->block<3, 1>(0, 3).eval() - dis*_planeVector[0];
	}
	catch (...){

	}
}


void Calibration::OnBnClickedButton34()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemText(IDC_EDIT1, cstring);
	try{
		double dis = stod(string(CT2A(cstring)));
		_transMatPtr->block<3, 1>(0, 3) = _transMatPtr->block<3, 1>(0, 3).eval() + dis*_planeVector[1];
	}
	catch (...){

	}
}


void Calibration::OnBnClickedButton35()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemText(IDC_EDIT1, cstring);
	try{
		double dis = stod(string(CT2A(cstring)));
		_transMatPtr->block<3, 1>(0, 3) = _transMatPtr->block<3, 1>(0, 3).eval() - dis*_planeVector[1];
	}
	catch (...){

	}
}


void Calibration::OnBnClickedButton36()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemText(IDC_EDIT1, cstring);
	try{
		double angle = stod(string(CT2A(cstring)))*M_PI/180;
		Matrix3d R;
		R = AngleAxisd(angle, _planeVector[2]);
		_transMatPtr->block<3, 3>(0, 0) = R*_transMatPtr->block<3, 3>(0, 0).eval();
	}
	catch (...){

	}
}


void Calibration::OnBnClickedButton37()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemText(IDC_EDIT1, cstring);
	try{
		double angle = stod(string(CT2A(cstring)))*M_PI / 180;
		Matrix3d R;
		R = AngleAxisd(-angle, _planeVector[2]);
		_transMatPtr->block<3, 3>(0, 0) = R*_transMatPtr->block<3, 3>(0, 0).eval();
	}
	catch (...){

	}
}


void Calibration::OnBnClickedButton5()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_thread){
		_thread->join();
		delete _thread;
	}
	_thread = new thread([=]{viewerWColor(*_transMatPtr, _cloud0, _cloud1); });
}

void Calibration::OnBnClickedButton1()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	// set TransMat
	_kinect1->setTransMat(Matrix4d::Identity());
	vector<double> leafSize{ 0.005, 0.005, 0.005 };
	PointCloud<PointXYZ>::Ptr kinectCloud0(new PointCloud<PointXYZ>), kinectCloud1(new PointCloud<PointXYZ>);
	kinectCloud0 = downSampling(_kinect0->getPointCloud(), leafSize);
	kinectCloud1 = downSampling(_kinect1->getPointCloud(), leafSize);
	io::savePLYFile("RawData/kinect0.ply", *kinectCloud0);
	io::savePLYFile("RawData/kinect1.ply", *kinectCloud1);
}

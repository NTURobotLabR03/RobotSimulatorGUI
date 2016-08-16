
// RobotSimulatorGUIDlg.cpp : 實作檔
//

#include "stdafx.h"
#include "RobotSimulatorGUI.h"
#include "RobotSimulatorGUIDlg.h"
#include "afxdialogex.h"
// TODO
#include "trajPlan.h"
#include "ThreadControl.h"
#include "FreeGlutCallback.h"
#include <fstream>
#include <string>
#include <sstream>
using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace tp;

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// 對 App About 使用 CAboutDlg 對話方塊

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 對話方塊資料
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

// 程式碼實作
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRobotSimulatorGUIDlg 對話方塊



CRobotSimulatorGUIDlg::CRobotSimulatorGUIDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRobotSimulatorGUIDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotSimulatorGUIDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_AABB, AABB_btn);
	DDX_Control(pDX, IDC_CONTACT, contact_btn);
	DDX_Control(pDX, IDC_RRT, rrt_btn);
	DDX_Control(pDX, IDC_PATH, path_btn);
	DDX_Control(pDX, IDC_COMBO2, m_cbObject);
	DDX_Control(pDX, IDC_CHECK1, m_IKenable);
	DDX_Control(pDX, IDC_COMBO3, m_cbTransMat);
}

BEGIN_MESSAGE_MAP(CRobotSimulatorGUIDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_GRASPPLANNING, &CRobotSimulatorGUIDlg::OnBnClickedGraspplanning)
	ON_BN_CLICKED(IDC_PATHPLANNING, &CRobotSimulatorGUIDlg::OnBnClickedPathplanning)
	ON_BN_CLICKED(IDC_PATHSMOOTHING, &CRobotSimulatorGUIDlg::OnBnClickedPathsmoothing)
	ON_BN_CLICKED(IDC_AABB, &CRobotSimulatorGUIDlg::OnBnClickedAabb)
	ON_BN_CLICKED(IDC_CONTACT, &CRobotSimulatorGUIDlg::OnBnClickedContact)
	ON_BN_CLICKED(IDC_RRT, &CRobotSimulatorGUIDlg::OnBnClickedRrt)
	ON_BN_CLICKED(IDC_PATH, &CRobotSimulatorGUIDlg::OnBnClickedPath)
	ON_BN_CLICKED(IDC_BUTTON1, &CRobotSimulatorGUIDlg::OnBnClickedButton1)
	ON_EN_CHANGE(IDC_EDIT1, &CRobotSimulatorGUIDlg::OnEnChangeEdit1)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN1, &CRobotSimulatorGUIDlg::OnDeltaposSpin1)
	ON_EN_CHANGE(IDC_EDIT3, &CRobotSimulatorGUIDlg::OnEnChangeEdit3)
	ON_EN_CHANGE(IDC_EDIT2, &CRobotSimulatorGUIDlg::OnEnChangeEdit2)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN3, &CRobotSimulatorGUIDlg::OnDeltaposSpin3)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN2, &CRobotSimulatorGUIDlg::OnDeltaposSpin2)
	ON_BN_CLICKED(IDC_BUTTON2, &CRobotSimulatorGUIDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CRobotSimulatorGUIDlg::OnBnClickedButton3)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CRobotSimulatorGUIDlg::OnCbnSelchangeCombo2)
	ON_EN_CHANGE(IDC_OBJECTX, &CRobotSimulatorGUIDlg::OnEnChangeObjectx)
	ON_EN_CHANGE(IDC_OBJECTY, &CRobotSimulatorGUIDlg::OnEnChangeObjecty)
	ON_EN_CHANGE(IDC_OBJECTZ, &CRobotSimulatorGUIDlg::OnEnChangeObjectz)
	ON_EN_CHANGE(IDC_OBJECTRX, &CRobotSimulatorGUIDlg::OnEnChangeObjectrx)
	ON_EN_CHANGE(IDC_OBJECTRY, &CRobotSimulatorGUIDlg::OnEnChangeObjectry)
	ON_EN_CHANGE(IDC_OBJECTRZ, &CRobotSimulatorGUIDlg::OnEnChangeObjectrz)
	ON_BN_CLICKED(IDC_BUTTON4, &CRobotSimulatorGUIDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CRobotSimulatorGUIDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &CRobotSimulatorGUIDlg::OnBnClickedButton6)
	ON_EN_CHANGE(IDC_ARM0, &CRobotSimulatorGUIDlg::OnEnChangeArm0)
	ON_EN_CHANGE(IDC_ARM1, &CRobotSimulatorGUIDlg::OnEnChangeArm1)
	ON_EN_CHANGE(IDC_ARM2, &CRobotSimulatorGUIDlg::OnEnChangeArm2)
	ON_EN_CHANGE(IDC_ARM3, &CRobotSimulatorGUIDlg::OnEnChangeArm3)
	ON_EN_CHANGE(IDC_ARM4, &CRobotSimulatorGUIDlg::OnEnChangeArm4)
	ON_EN_CHANGE(IDC_ARM5, &CRobotSimulatorGUIDlg::OnEnChangeArm5)
	ON_BN_CLICKED(IDC_Qinit, &CRobotSimulatorGUIDlg::OnBnClickedQinit)
	ON_BN_CLICKED(IDC_Qgoal, &CRobotSimulatorGUIDlg::OnBnClickedQgoal)
	ON_BN_CLICKED(IDC_BUTTON8, &CRobotSimulatorGUIDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON9, &CRobotSimulatorGUIDlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_BUTTON11, &CRobotSimulatorGUIDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON10, &CRobotSimulatorGUIDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_CHECK1, &CRobotSimulatorGUIDlg::OnBnClickedCheck1)
	ON_EN_CHANGE(IDC_IKX, &CRobotSimulatorGUIDlg::OnEnChangeIkx)
	ON_EN_CHANGE(IDC_IKY, &CRobotSimulatorGUIDlg::OnEnChangeIky)
	ON_EN_CHANGE(IDC_IKZ, &CRobotSimulatorGUIDlg::OnEnChangeIkz)
	ON_EN_CHANGE(IDC_IKRX, &CRobotSimulatorGUIDlg::OnEnChangeIkrx)
	ON_EN_CHANGE(IDC_IKRY, &CRobotSimulatorGUIDlg::OnEnChangeIkry)
	ON_EN_CHANGE(IDC_IKRZ, &CRobotSimulatorGUIDlg::OnEnChangeIkrz)
	ON_BN_CLICKED(IDC_BUTTON12, &CRobotSimulatorGUIDlg::OnBnClickedButton12)
	ON_BN_CLICKED(IDC_BUTTON13, &CRobotSimulatorGUIDlg::OnBnClickedButton13)
	ON_BN_CLICKED(IDC_BUTTON14, &CRobotSimulatorGUIDlg::OnBnClickedButton14)
	ON_BN_CLICKED(IDC_BUTTON15, &CRobotSimulatorGUIDlg::OnBnClickedButton15)
	ON_BN_CLICKED(IDC_BUTTON16, &CRobotSimulatorGUIDlg::OnBnClickedButton16)
	ON_BN_CLICKED(IDC_BUTTON17, &CRobotSimulatorGUIDlg::OnBnClickedButton17)
	ON_BN_CLICKED(IDC_BUTTON18, &CRobotSimulatorGUIDlg::OnBnClickedButton18)
	ON_BN_CLICKED(IDC_BUTTON19, &CRobotSimulatorGUIDlg::OnBnClickedButton19)
	ON_BN_CLICKED(IDC_BUTTON20, &CRobotSimulatorGUIDlg::OnBnClickedButton20)
	ON_BN_CLICKED(IDC_BUTTON21, &CRobotSimulatorGUIDlg::OnBnClickedButton21)
	ON_BN_CLICKED(IDC_BUTTON22, &CRobotSimulatorGUIDlg::OnBnClickedButton22)
	ON_BN_CLICKED(IDC_BUTTON23, &CRobotSimulatorGUIDlg::OnBnClickedButton23)
	ON_BN_CLICKED(IDC_BUTTON24, &CRobotSimulatorGUIDlg::OnBnClickedButton24)
	ON_BN_CLICKED(IDC_BUTTON25, &CRobotSimulatorGUIDlg::OnBnClickedButton25)
	ON_BN_CLICKED(IDC_BUTTON26, &CRobotSimulatorGUIDlg::OnBnClickedButton26)
	ON_BN_CLICKED(IDC_BUTTON27, &CRobotSimulatorGUIDlg::OnBnClickedButton27)
	ON_BN_CLICKED(IDC_BUTTON30, &CRobotSimulatorGUIDlg::OnBnClickedButton30)
	ON_EN_CHANGE(IDC_EDIT4, &CRobotSimulatorGUIDlg::OnEnChangeEdit4)
	ON_EN_CHANGE(IDC_EDIT5, &CRobotSimulatorGUIDlg::OnEnChangeEdit5)
	ON_EN_CHANGE(IDC_EDIT6, &CRobotSimulatorGUIDlg::OnEnChangeEdit6)
	ON_EN_CHANGE(IDC_EDIT7, &CRobotSimulatorGUIDlg::OnEnChangeEdit7)
	ON_EN_CHANGE(IDC_EDIT8, &CRobotSimulatorGUIDlg::OnEnChangeEdit8)
	ON_EN_CHANGE(IDC_EDIT9, &CRobotSimulatorGUIDlg::OnEnChangeEdit9)
	ON_CBN_SELCHANGE(IDC_COMBO3, &CRobotSimulatorGUIDlg::OnCbnSelchangeCombo3)
	ON_EN_CHANGE(IDC_TRANSX, &CRobotSimulatorGUIDlg::OnEnChangeTransx)
	ON_EN_CHANGE(IDC_TRANSY, &CRobotSimulatorGUIDlg::OnEnChangeTransy)
	ON_EN_CHANGE(IDC_TRANSZ, &CRobotSimulatorGUIDlg::OnEnChangeTransz)
	ON_EN_CHANGE(IDC_TRANSRX, &CRobotSimulatorGUIDlg::OnEnChangeTransrx)
	ON_EN_CHANGE(IDC_TRANSRY, &CRobotSimulatorGUIDlg::OnEnChangeTransry)
	ON_EN_CHANGE(IDC_TRANSRZ, &CRobotSimulatorGUIDlg::OnEnChangeTransrz)
	ON_BN_CLICKED(IDC_BUTTON28, &CRobotSimulatorGUIDlg::OnBnClickedButton28)
	ON_BN_CLICKED(IDC_BUTTON29, &CRobotSimulatorGUIDlg::OnBnClickedButton29)
	ON_BN_CLICKED(IDC_BUTTON31, &CRobotSimulatorGUIDlg::OnBnClickedButton31)
	ON_BN_CLICKED(IDC_BUTTON32, &CRobotSimulatorGUIDlg::OnBnClickedButton32)
END_MESSAGE_MAP()


// CRobotSimulatorGUIDlg 訊息處理常式

BOOL CRobotSimulatorGUIDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 將 [關於...] 功能表加入系統功能表。

	// IDM_ABOUTBOX 必須在系統命令範圍之中。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 設定此對話方塊的圖示。當應用程式的主視窗不是對話方塊時，
	// 框架會自動從事此作業
	SetIcon(m_hIcon, TRUE);			// 設定大圖示
	SetIcon(m_hIcon, FALSE);		// 設定小圖示

	// TODO:  在此加入額外的初始設定
	// 開simulator 的thread
	_demo = new UISimulator();
	int argc = 1;
	char** argv = new char*("openGL");
	_simulatorThread = thread([=]{glutmain(argc, argv, 800, 600, "Robot Simulator", _demo); });
	// 讀取table位置
	fstream fin;
	fin.open("parameter/TablePosition", ios::in);
	if (!fin){
		_tablePosition << 830, 0, -130;
	}
	else{
		fin >> _tablePosition[0] >> _tablePosition[1] >> _tablePosition[2];
		fin.close();
	}
	ostringstream os;
	os << _tablePosition[0];
	SetDlgItemTextW(IDC_EDIT3, CString(os.str().c_str()));
	os.str("");
	os << _tablePosition[1];
	SetDlgItemTextW(IDC_EDIT2, CString(os.str().c_str()));
	os.str("");
	os << _tablePosition[2];
	SetDlgItemTextW(IDC_EDIT1, CString(os.str().c_str()));
	{
		lock_guard<mutex> mLock(gMutex);
		_demo->getTable()->setCOM3D(_tablePosition);
	}
	
	_armJointValues = { 0, 0, 0, 0, 0, 0 };
	SetDlgItemTextW(IDC_ARM0, CString("0"));
	SetDlgItemTextW(IDC_ARM1, CString("0"));
	SetDlgItemTextW(IDC_ARM2, CString("0"));
	SetDlgItemTextW(IDC_ARM3, CString("0"));
	SetDlgItemTextW(IDC_ARM4, CString("0"));
	SetDlgItemTextW(IDC_ARM5, CString("0"));

	// open 放下物體地點 place information
	_placeJointValues = { 0, 0, 0, 0, 0, 0 };
	fin.open("parameter/PlaceInformation", ios::in);
	if (!fin){

	}
	else{
		fin >> _placeJointValues[0] >> _placeJointValues[1] >> _placeJointValues[2] >> _placeJointValues[3] >> _placeJointValues[4] >> _placeJointValues[5];
		fin.close();
	}
	((CEdit*)GetDlgItem(IDC_IKX))->SetReadOnly(true);
	((CEdit*)GetDlgItem(IDC_IKY))->SetReadOnly(true);
	((CEdit*)GetDlgItem(IDC_IKZ))->SetReadOnly(true);
	((CEdit*)GetDlgItem(IDC_IKRX))->SetReadOnly(true);
	((CEdit*)GetDlgItem(IDC_IKRY))->SetReadOnly(true);
	((CEdit*)GetDlgItem(IDC_IKRZ))->SetReadOnly(true);
	((CButton*)GetDlgItem(IDC_BUTTON12))->EnableWindow(false);


	OnBnClickedQinit();
	
	// PCL 初始化
	fin.open("parameter/KinectTransMat", ios::in);
	if (!fin){
		_transMat = Matrix4d::Identity();
	}
	else{
		for (int i = 0; i < 4; ++i){
			for (int j = 0; j < 4; ++j){
				fin >> _transMat(i, j);
			}
		}
		fin.close();
	}

	cout << "The relation between Kinects:" << endl;
	cout << _transMat << endl;

	fin.open("parameter/TransMat2Base", ios::in);
	if (!fin){
		_T2base = Matrix4d::Identity();
	}
	else{
		for (int i = 0; i < 4; ++i){
			for (int j = 0; j < 4; ++j){
				fin >> _T2base(i, j);
			}
		}
		fin.close();
	}

	cout << endl;
	cout << "The relation from kinect 2 Base:" << endl;
	cout << _T2base << endl;

	_xlimits = { -2.0, 2.0 };
	_ylimits = { -2.0, 2.0 };
	_zlimits = { -2.0, 2.0 };
	fin.open("parameter/PathThroughParameters", ios::in);
	if (!fin){

	}
	else{
		fin >> _xlimits[0] >> _xlimits[1] >> _ylimits[0] >> _ylimits[1] >> _zlimits[0] >> _zlimits[1];
		fin.close();
	}

	os.str("");
	CString cstring;
	os << _xlimits[0];
	cstring = os.str().c_str();
	SetDlgItemTextW(IDC_EDIT4, cstring);
	os.str("");
	os << _ylimits[0];
	cstring = os.str().c_str();
	SetDlgItemTextW(IDC_EDIT6, cstring);
	os.str("");
	os << _zlimits[0];
	cstring = os.str().c_str();
	SetDlgItemTextW(IDC_EDIT8, cstring);
	os.str("");
	os << _xlimits[1];
	cstring = os.str().c_str();
	SetDlgItemTextW(IDC_EDIT5, cstring);
	os.str("");
	os << _ylimits[1];
	cstring = os.str().c_str();
	SetDlgItemTextW(IDC_EDIT7, cstring);
	os.str("");
	os << _zlimits[1];
	cstring = os.str().c_str();
	SetDlgItemTextW(IDC_EDIT9, cstring);
	os.str("");

	m_cbTransMat.AddString(CString("Kinect2Kinect"));
	m_cbTransMat.AddString(CString("PCL2Base"));

	return TRUE;  // 傳回 TRUE，除非您對控制項設定焦點
}

void CRobotSimulatorGUIDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果將最小化按鈕加入您的對話方塊，您需要下列的程式碼，
// 以便繪製圖示。對於使用文件/檢視模式的 MFC 應用程式，
// 框架會自動完成此作業。

void CRobotSimulatorGUIDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 繪製的裝置內容

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 將圖示置中於用戶端矩形
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 描繪圖示
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 當使用者拖曳最小化視窗時，
// 系統呼叫這個功能取得游標顯示。
HCURSOR CRobotSimulatorGUIDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CRobotSimulatorGUIDlg::OnBnClickedGraspplanning()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	gMutex.lock();
	int maxIter = 10;
	double maxScore = 0.0;
	vector<double> handTh;
	for (int i = 0; i < maxIter; ++i){
		double score = _demo->graspPlanning();
		if (score > maxScore){
			maxScore = score;
			_demo->getArmJoint(_armJointValues);
			_demo->getHandJoint(handTh);
			if (maxScore > 0.03){
				break;
			}
		}
		else if (score == -1.0){
			break;
		}
	}
	_demo->setHandJoint(handTh);
	gMutex.unlock();
	cout << maxScore << endl;
	_targetPosMat = _demo->getTarget()->getCOM6D();
	lock_guard<mutex> mLock(gMutex);
	ostringstream os;
	os << _armJointValues[0] *180/M_PI;
	SetDlgItemTextW(IDC_ARM0, CString(os.str().c_str()));
	os.str("");
	os << _armJointValues[1] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM1, CString(os.str().c_str()));
	os.str("");
	os << _armJointValues[2] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM2, CString(os.str().c_str()));
	os.str("");
	os << _armJointValues[3] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM3, CString(os.str().c_str()));
	os.str("");
	os << _armJointValues[4] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM4, CString(os.str().c_str()));
	os.str("");
	os << _armJointValues[5] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM5, CString(os.str().c_str()));
	_isGrasp = true;
	OnBnClickedQgoal();
}


void CRobotSimulatorGUIDlg::OnBnClickedPathplanning()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_demo->pathPlanning(_Qinit, _Qgoal);
	if (_Qgoal.getIsGrasp()){
		if (!_Qinit.getIsGrasp()){
			_demo->handGrasp();
		}
		_isGrasp = true;
		if (_demo->getTarget()->getTempDeleteFlag()){
			_demo->getTarget()->setTempDeleteFlag(false);
		}
	}
	else{
		_demo->handRelease();
		_isGrasp = false;
		if (_demo->getTarget()->getTempDeleteFlag()){
			_demo->tempDeleteObject(_demo->getTarget());
		}
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedPathsmoothing()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_Qinit.getIsGrasp()){
		_paths.push_back(new PathPlayBack(_Qinit, _Qgoal, _demo->getPath(), _demo->getTarget(), _targetPosMat));
		_targetPosMat = _demo->getTarget()->getCOM6D();
	}
	else{
		_paths.push_back(new PathPlayBack(_Qinit, _Qgoal, _demo->getPath()));
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedAabb()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	static bool click = true;
	if (click)
		AABB_btn.SetWindowTextW(_T("AABB  ON"));
	else
		AABB_btn.SetWindowTextW(_T("AABB  OFF"));
	click = !click;
	_demo->setDrawAabbFlag();
}


void CRobotSimulatorGUIDlg::OnBnClickedContact()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	static bool click = true;
	if (click)
		contact_btn.SetWindowTextW(_T("Contact  ON"));
	else
		contact_btn.SetWindowTextW(_T("Contact  OFF"));
	_demo->setContactMgrDrawFlag(click);
	click = !click;
}


void CRobotSimulatorGUIDlg::OnBnClickedRrt()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	static bool click = true;
	if (click)
		rrt_btn.SetWindowTextW(_T("RRT  ON"));
	else
		rrt_btn.SetWindowTextW(_T("RRT  OFF"));
	_demo->setRRTDrawFlag(click);
	click = !click;
}


void CRobotSimulatorGUIDlg::OnBnClickedPath()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	static bool click = true;
	if (click)
		path_btn.SetWindowTextW(_T("Path  ON"));
	else
		path_btn.SetWindowTextW(_T("Path  OFF"));
	_demo->setPathFlag(click);
	click = !click;
}


void CRobotSimulatorGUIDlg::OnBnClickedButton1()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (!_isGrasp){
		_demo->handGrasp();
	}
	_isGrasp = true;
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit1()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnDeltaposSpin1(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO:  在此加入控制項告知處理常式程式碼
	if (pNMUpDown->iDelta == 1){ // 按下往下
		_tablePosition[2]--;
	}
	else if(pNMUpDown->iDelta == -1){ // 按下往上
		_tablePosition[2]++;
	}
	_demo->getTable()->setCOM3D(_tablePosition);
	ostringstream os;
	os << _tablePosition[2];
	SetDlgItemTextW(IDC_EDIT1, CString(os.str().c_str()));
	fstream fout;
	fout.open("parameter/TablePosition", ios::out);
	for (int i = 0, n = _tablePosition.size(); i < n; ++i){
		fout << _tablePosition[i] << endl;
	}
	fout.close();
	*pResult = 0;
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit3()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit2()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnDeltaposSpin3(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO:  在此加入控制項告知處理常式程式碼
	if (pNMUpDown->iDelta == 1){ // 按下往下
		_tablePosition[0]--;
	}
	else if (pNMUpDown->iDelta == -1){ // 按下往上
		_tablePosition[0]++;
	}
	_demo->getTable()->setCOM3D(_tablePosition);
	ostringstream os;
	os << _tablePosition[0];
	SetDlgItemTextW(IDC_EDIT3, CString(os.str().c_str()));
	fstream fout;
	fout.open("parameter/TablePosition", ios::out);
	for (int i = 0, n = _tablePosition.size(); i < n; ++i){
		fout << _tablePosition[i] << endl;
	}
	fout.close();
	*pResult = 0;
}


void CRobotSimulatorGUIDlg::OnDeltaposSpin2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO:  在此加入控制項告知處理常式程式碼
	if (pNMUpDown->iDelta == 1){ // 按下往下
		_tablePosition[1]--;
	}
	else if (pNMUpDown->iDelta == -1){ // 按下往上
		_tablePosition[1]++;
	}
	_demo->getTable()->setCOM3D(_tablePosition);
	ostringstream os;
	os << _tablePosition[1];
	SetDlgItemTextW(IDC_EDIT2, CString(os.str().c_str()));
	fstream fout;
	fout.open("parameter/TablePosition", ios::out);
	for (int i = 0, n = _tablePosition.size(); i < n; ++i){
		fout << _tablePosition[i] << endl;
	}
	fout.close();
	*pResult = 0;
}


void CRobotSimulatorGUIDlg::OnBnClickedButton2()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	//過濾副檔名
	CString szFilter = CString("Object File|*.obj|All File|*.*||");
	//以Dialog方式開啟檔案
	CFileDialog fd(true, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
	if (fd.DoModal() == IDOK)  //按下OK 
	{
		gMutex.lock();
		for (auto& it : _objects){
			it->setColor(btVector3(1.0, 1.0, 1.0));
		}
		//開啟檔案成功
		CString szFileName = fd.GetPathName(); //取得開啟檔案的全名(包含路徑)
		_objects.push_back(_demo->CreateObject(_demo->loadObjFile(CT2A(szFileName)), btVector3(1.0, 1.0, 0.0), Matrix4d::Identity(), COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		gMutex.unlock();
		m_cbObject.AddString(fd.GetFileTitle());
		m_cbObject.SetCurSel(_objects.size() - 1);
		_demo->setTarget(_objects.back());
		Matrix4d T;
		VectorXd p;
		T = _objects.back()->getCOM6D();
		rbt::transMat2Vec(T, p);
		ostringstream os;
		os << p[0];
		SetDlgItemTextW(IDC_OBJECTX, CString(os.str().c_str()));
		os.str("");
		os << p[1];
		SetDlgItemTextW(IDC_OBJECTY, CString(os.str().c_str()));
		os.str("");
		os << p[2];
		SetDlgItemTextW(IDC_OBJECTZ, CString(os.str().c_str()));
		os.str("");
		os << p[3] * 180 / M_PI;
		SetDlgItemTextW(IDC_OBJECTRX, CString(os.str().c_str()));
		os.str("");
		os << p[4] * 180 / M_PI;
		SetDlgItemTextW(IDC_OBJECTRY, CString(os.str().c_str()));
		os.str("");
		os << p[5] * 180 / M_PI;
		SetDlgItemTextW(IDC_OBJECTRZ, CString(os.str().c_str()));
		os.str("");
	}
	else  //按下Cancel
	{

	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton3()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (m_cbObject.GetCurSel() >= 0){
		lock_guard<mutex> mLock(gMutex);
		int index = m_cbObject.GetCurSel();
		_demo->deleteObject(_objects[index]);
		_objects.erase(_objects.begin() + index);
		m_cbObject.DeleteString(index);

		m_cbObject.SetCurSel(-1);

		SetDlgItemTextW(IDC_OBJECTX, CString(""));
		SetDlgItemTextW(IDC_OBJECTY, CString(""));
		SetDlgItemTextW(IDC_OBJECTZ, CString(""));
		SetDlgItemTextW(IDC_OBJECTRX, CString(""));
		SetDlgItemTextW(IDC_OBJECTRY, CString(""));
		SetDlgItemTextW(IDC_OBJECTRZ, CString(""));
		_demo->setTarget(nullptr);
	}
}


void CRobotSimulatorGUIDlg::OnCbnSelchangeCombo2()
{
	gMutex.lock();
	// TODO:  在此加入控制項告知處理常式程式碼
	for (auto& it : _objects){
		it->setColor(btVector3(1.0, 1.0, 1.0));
	}
	int index = m_cbObject.GetCurSel();
	_objects[index]->setColor(btVector3(1.0, 1.0, 0.0));
	gMutex.unlock();
	_demo->setTarget(_objects[index]);
	Matrix4d T;
	VectorXd p;
	T = _objects[index]->getCOM6D();
	rbt::transMat2Vec(T, p);
	ostringstream os;
	os << p[0];
	SetDlgItemTextW(IDC_OBJECTX, CString(os.str().c_str()));
	os.str("");
	os << p[1];
	SetDlgItemTextW(IDC_OBJECTY, CString(os.str().c_str()));
	os.str("");
	os << p[2];
	SetDlgItemTextW(IDC_OBJECTZ, CString(os.str().c_str()));
	os.str("");
	os << p[3] * 180 / M_PI;
	SetDlgItemTextW(IDC_OBJECTRX, CString(os.str().c_str()));
	os.str("");
	os << p[4] * 180 / M_PI;
	SetDlgItemTextW(IDC_OBJECTRY, CString(os.str().c_str()));
	os.str("");
	os << p[5] * 180 / M_PI;
	SetDlgItemTextW(IDC_OBJECTRZ, CString(os.str().c_str()));
	os.str("");
}


void CRobotSimulatorGUIDlg::OnEnChangeObjectx()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbObject.GetCurSel();
	if (index >= 0){
		CString cstring;
		GetDlgItemTextW(IDC_OBJECTX, cstring);
		Vector3d pos;
		pos = _objects[index]->getCOM3D();
		try{
			pos[0] = stod(string(CT2A(cstring)));
		}
		catch (...){
			pos[0] = 0;
		}
		gMutex.lock();
		_objects[index]->setCOM3D(pos);
		gMutex.unlock();
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeObjecty()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbObject.GetCurSel();
	if (index >= 0){
		CString cstring;
		GetDlgItemTextW(IDC_OBJECTY, cstring);
		Vector3d pos;
		pos = _objects[index]->getCOM3D();
		try{
			pos[1] = stod(string(CT2A(cstring)));
		}
		catch (...){
			pos[1] = 0;
		}
		gMutex.lock();
		_objects[index]->setCOM3D(pos);
		gMutex.unlock();
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeObjectz()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbObject.GetCurSel();
	if (index >= 0){
		CString cstring;
		GetDlgItemTextW(IDC_OBJECTZ, cstring);
		Vector3d pos;
		pos = _objects[index]->getCOM3D();
		try{
			pos[2] = stod(string(CT2A(cstring)));
		}
		catch (...){
			pos[2] = 0;
		}
		gMutex.lock();
		_objects[index]->setCOM3D(pos);
		gMutex.unlock();
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeObjectrx()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbObject.GetCurSel();
	if (index >= 0){
		CString cstring;
		GetDlgItemTextW(IDC_OBJECTRX, cstring);
		MatrixXd T;
		T = _objects[index]->getCOM6D();
		VectorXd p;
		rbt::transMat2Vec(T, p);
		Vector3d o;
		o = p.tail(3);
		try{
			o[0] = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			o[0] = 0;
		}
		Matrix3d R;
		kaxisTh2Rot(o, R);
		T.block<3, 3>(0, 0) = R;
		gMutex.lock();
		_objects[index]->setCOM6D(T);
		gMutex.unlock();
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeObjectry()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbObject.GetCurSel();
	if (index >= 0){
		CString cstring;
		GetDlgItemTextW(IDC_OBJECTRY, cstring);
		MatrixXd T;
		T = _objects[index]->getCOM6D();
		VectorXd p;
		rbt::transMat2Vec(T, p);
		Vector3d o;
		o = p.tail(3);
		try{
			o[1] = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			o[1] = 0;
		}
		Matrix3d R;
		kaxisTh2Rot(o, R);
		T.block<3, 3>(0, 0) = R;
		gMutex.lock();
		_objects[index]->setCOM6D(T);
		gMutex.unlock();
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeObjectrz()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbObject.GetCurSel();
	if (index >= 0){
		CString cstring;
		GetDlgItemTextW(IDC_OBJECTRZ, cstring);
		MatrixXd T;
		T = _objects[index]->getCOM6D();
		VectorXd p;
		rbt::transMat2Vec(T, p);
		Vector3d o;
		o = p.tail(3);
		try{
			o[2] = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			o[2] = 0;
		}
		Matrix3d R;
		kaxisTh2Rot(o, R);
		T.block<3, 3>(0, 0) = R;
		gMutex.lock();
		_objects[index]->setCOM6D(T);
		gMutex.unlock();
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnBnClickedButton4()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (m_cbObject.GetCurSel() >= 0){
		//過濾副檔名
		CString szFilter = CString("Pose File|*.pos||");
		//以Dialog方式存檔案
		CFileDialog fd(false, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
		if (fd.DoModal() == IDOK)  //按下OK 
		{
			string fileName = CT2A(fd.GetPathName()) + ".pos";
			fstream fout;
			int index = m_cbObject.GetCurSel();
			fout.open(fileName, ios::out);
			fout << _objects[index]->getCOM6D() << endl;
			fout.close();
		}
		else{ //按下Cancel

		}
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton5()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (m_cbObject.GetCurSel() >= 0){
		//過濾副檔名
		CString szFilter = CString("Pose File|*.pos|All File|*.*||");
		//以Dialog方式開啟檔案
		CFileDialog fd(true, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
		if (fd.DoModal() == IDOK)  //按下OK 
		{
			Matrix4d T;
			fstream fin;
			fin.open(CT2A(fd.GetPathName()), ios::in);
			for (int i = 0, n = T.size(); i < n; ++i){
				fin >> T.data()[i];
			}
			fin.close();
			T.transposeInPlace();
			int index = m_cbObject.GetCurSel();
			_objects[index]->setCOM6D(T);
			m_cbObject.SetCurSel(index);
			VectorXd p;
			rbt::transMat2Vec(T, p);
			ostringstream os;
			os << p[0];
			SetDlgItemTextW(IDC_OBJECTX, CString(os.str().c_str()));
			os.str("");
			os << p[1];
			SetDlgItemTextW(IDC_OBJECTY, CString(os.str().c_str()));
			os.str("");
			os << p[2];
			SetDlgItemTextW(IDC_OBJECTZ, CString(os.str().c_str()));
			os.str("");
			os << p[3] * 180 / M_PI;
			SetDlgItemTextW(IDC_OBJECTRX, CString(os.str().c_str()));
			os.str("");
			os << p[4] * 180 / M_PI;
			SetDlgItemTextW(IDC_OBJECTRY, CString(os.str().c_str()));
			os.str("");
			os << p[5] * 180 / M_PI;
			SetDlgItemTextW(IDC_OBJECTRZ, CString(os.str().c_str()));
			os.str("");
		}
		else{ //按下Cancel

		}
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton6()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_demo->handRelease();
	_isGrasp = false;
}


void CRobotSimulatorGUIDlg::OnEnChangeArm0()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_ARM0, cstring);
	double value = 0;
	try{
		value = stod(string(CT2A(cstring)))*M_PI/180;
	}
	catch (...){
		
	}
	_armJointValues[0] = value;
	_demo->setArmJoint(_armJointValues);
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeArm1()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_ARM1, cstring);
	double value = 0;
	try{
		value = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){

	}
	_armJointValues[1] = value;
	_demo->setArmJoint(_armJointValues);
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeArm2()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_ARM2, cstring);
	double value = 0;
	try{
		value = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){

	}
	_armJointValues[2] = value;
	_demo->setArmJoint(_armJointValues);
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeArm3()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_ARM3, cstring);
	double value = 0;
	try{
		value = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){

	}
	_armJointValues[3] = value;
	_demo->setArmJoint(_armJointValues);
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeArm4()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_ARM4, cstring);
	double value = 0;
	try{
		value = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){

	}
	_armJointValues[4] = value;
	_demo->setArmJoint(_armJointValues);
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeArm5()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_ARM5, cstring);
	double value = 0;
	try{
		value = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){

	}
	_armJointValues[5] = value;
	_demo->setArmJoint(_armJointValues);
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnBnClickedQinit()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_demo->getArmJoint(_Qinit.getArmTh());
	_demo->getHandJoint(_Qinit.getHandTh());
	_Qinit.setIsGrasp(_isGrasp);
}


void CRobotSimulatorGUIDlg::OnBnClickedQgoal()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_demo->getArmJoint(_Qgoal.getArmTh());
	_demo->getHandJoint(_Qgoal.getHandTh());
	_Qgoal.setIsGrasp(_isGrasp);
}


void CRobotSimulatorGUIDlg::OnBnClickedButton8()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	SetDlgItemTextW(IDC_ARM0, CString("0"));
	SetDlgItemTextW(IDC_ARM1, CString("0"));
	SetDlgItemTextW(IDC_ARM2, CString("0"));
	SetDlgItemTextW(IDC_ARM3, CString("0"));
	SetDlgItemTextW(IDC_ARM4, CString("0"));
	SetDlgItemTextW(IDC_ARM5, CString("0"));
}


void CRobotSimulatorGUIDlg::OnBnClickedButton9()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	cout << _demo->checkCollision() << endl;
}


void CRobotSimulatorGUIDlg::OnBnClickedButton11()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_placeJointValues = _armJointValues;
	fstream fout;
	fout.open("parameter/PlaceInformation", ios::out);
	for (auto& it : _placeJointValues){
		fout << it << endl;
	}
	fout.close();
}


void CRobotSimulatorGUIDlg::OnBnClickedButton10()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	lock_guard<mutex> mLock(gMutex);
	ostringstream os;
	os << _placeJointValues[0] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM0, CString(os.str().c_str()));
	os.str("");
	os << _placeJointValues[1] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM1, CString(os.str().c_str()));
	os.str("");
	os << _placeJointValues[2] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM2, CString(os.str().c_str()));
	os.str("");
	os << _placeJointValues[3] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM3, CString(os.str().c_str()));
	os.str("");
	os << _placeJointValues[4] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM4, CString(os.str().c_str()));
	os.str("");
	os << _placeJointValues[5] * 180 / M_PI;
	SetDlgItemTextW(IDC_ARM5, CString(os.str().c_str()));

	if (_demo->getTarget()){
		_demo->getTarget()->setTempDeleteFlag(true);
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedCheck1()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (m_IKenable.GetCheck()){
		((CEdit*)GetDlgItem(IDC_IKX))->SetReadOnly(false);
		((CEdit*)GetDlgItem(IDC_IKY))->SetReadOnly(false);
		((CEdit*)GetDlgItem(IDC_IKZ))->SetReadOnly(false);
		((CEdit*)GetDlgItem(IDC_IKRX))->SetReadOnly(false);
		((CEdit*)GetDlgItem(IDC_IKRY))->SetReadOnly(false);
		((CEdit*)GetDlgItem(IDC_IKRZ))->SetReadOnly(false);
		((CButton*)GetDlgItem(IDC_BUTTON12))->EnableWindow(true);
		_demo->setAidFlag(true);
		_demo->getArmFK(_demo->getAidFrame());
		VectorXd p;
		rbt::transMat2Vec(_demo->getAidFrame(), p);
		ostringstream os;
		os << p[0];
		SetDlgItemTextW(IDC_IKX, CString(os.str().c_str()));
		os.str("");
		os << p[1];
		SetDlgItemTextW(IDC_IKY, CString(os.str().c_str()));
		os.str("");
		os << p[2];
		SetDlgItemTextW(IDC_IKZ, CString(os.str().c_str()));
		os.str("");
		os << p[3] * 180 / M_PI;
		SetDlgItemTextW(IDC_IKRX, CString(os.str().c_str()));
		os.str("");
		os << p[4] * 180 / M_PI;
		SetDlgItemTextW(IDC_IKRY, CString(os.str().c_str()));
		os.str("");
		os << p[5] * 180 / M_PI;
		SetDlgItemTextW(IDC_IKRZ, CString(os.str().c_str()));
		os.str("");
	}
	else{
		((CEdit*)GetDlgItem(IDC_IKX))->SetReadOnly(true);
		((CEdit*)GetDlgItem(IDC_IKY))->SetReadOnly(true);
		((CEdit*)GetDlgItem(IDC_IKZ))->SetReadOnly(true);
		((CEdit*)GetDlgItem(IDC_IKRX))->SetReadOnly(true);
		((CEdit*)GetDlgItem(IDC_IKRY))->SetReadOnly(true);
		((CEdit*)GetDlgItem(IDC_IKRZ))->SetReadOnly(true);
		((CButton*)GetDlgItem(IDC_BUTTON12))->EnableWindow(false);
		_demo->setAidFlag(false);
		SetDlgItemTextW(IDC_IKX, CString(""));
		SetDlgItemTextW(IDC_IKY, CString(""));
		SetDlgItemTextW(IDC_IKZ, CString(""));
		SetDlgItemTextW(IDC_IKRX, CString(""));
		SetDlgItemTextW(IDC_IKRY, CString(""));
		SetDlgItemTextW(IDC_IKRZ, CString(""));
	}
}


void CRobotSimulatorGUIDlg::OnEnChangeIkx()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_IKX, cstring);
	gMutex.lock();
	try{
		_demo->getAidFrame()(0, 3) = stod(string(CT2A(cstring)));
	}
	catch(...){
		_demo->getAidFrame()(0, 3) = 0;
	}
	gMutex.unlock();
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeIky()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_IKY, cstring);
	gMutex.lock();
	try{
		_demo->getAidFrame()(1, 3) = stod(string(CT2A(cstring)));
	}
	catch (...){
		_demo->getAidFrame()(1, 3) = 0;
	}
	gMutex.unlock();
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeIkz()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_IKZ, cstring);
	gMutex.lock();
	try{
		_demo->getAidFrame()(2, 3) = stod(string(CT2A(cstring)));
	}
	catch (...){
		_demo->getAidFrame()(2, 3) = 0;
	}
	gMutex.unlock();
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeIkrx()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_IKRX, cstring);
	VectorXd p;
	rbt::transMat2Vec(_demo->getAidFrame(), p);
	Vector3d o;
	o = p.tail(3);
	try{
		o[0] = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){
		o[0] = 0;
	}
	Matrix3d R;
	kaxisTh2Rot(o, R);
	_demo->getAidFrame().block<3, 3>(0, 0) = R;
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeIkry()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_IKRY, cstring);
	VectorXd p;
	rbt::transMat2Vec(_demo->getAidFrame(), p);
	Vector3d o;
	o = p.tail(3);
	try{
		o[1] = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){
		o[1] = 0;
	}
	Matrix3d R;
	kaxisTh2Rot(o, R);
	_demo->getAidFrame().block<3, 3>(0, 0) = R;
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeIkrz()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	GetDlgItemTextW(IDC_IKRZ, cstring);
	VectorXd p;
	rbt::transMat2Vec(_demo->getAidFrame(), p);
	Vector3d o;
	o = p.tail(3);
	try{
		o[2] = stod(string(CT2A(cstring)))*M_PI / 180;
	}
	catch (...){
		o[2] = 0;
	}
	Matrix3d R;
	kaxisTh2Rot(o, R);
	_demo->getAidFrame().block<3, 3>(0, 0) = R;
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnBnClickedButton12()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	lock_guard<mutex> mLock(gMutex);
	_demo->solveIK();
	vector<double> th;
	stringstream os;
	_demo->getArmJoint(th);
	os << th[0] * 180 / M_PI;
	SetDlgItemText(IDC_ARM0, CString(os.str().c_str()));
	os.str("");
	os << th[1] * 180 / M_PI;
	SetDlgItemText(IDC_ARM1, CString(os.str().c_str()));
	os.str("");
	os << th[2] * 180 / M_PI;
	SetDlgItemText(IDC_ARM2, CString(os.str().c_str()));
	os.str("");
	os << th[3] * 180 / M_PI;
	SetDlgItemText(IDC_ARM3, CString(os.str().c_str()));
	os.str("");
	os << th[4] * 180 / M_PI;
	SetDlgItemText(IDC_ARM4, CString(os.str().c_str()));
	os.str("");
	os << th[5] * 180 / M_PI;
	SetDlgItemText(IDC_ARM5, CString(os.str().c_str()));
	os.str("");
}


void CRobotSimulatorGUIDlg::OnBnClickedButton13()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	gMutex.lock();
	for (auto& it : _objects){
		if (it->getTempDeleteFlag()){
			_demo->restoreObject(it);
			it->setTempDeleteFlag(false);
		}
	}
	for (int i = _paths.size() - 1; i >= 0; --i){
		if (_paths[i]->getObj()){
			_paths[i]->getObj()->setCOM6D(_paths[i]->getObjT0());
		}
	}
	gMutex.unlock();
	for (auto& it : _paths){
		_demo->runPath(it);
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton14()
{
	_paths.clear();
	// TODO:  在此加入控制項告知處理常式程式碼
	for (int i = 0, n = _objects.size(); i < n; ++i){
		string token = "r";
		for (auto& it : _objects){
			it->setColor(btVector3(1.0, 1.0, 1.0));
		}
		_objects[i]->setColor(btVector3(1.0, 1.0, 0.0));
		_demo->setTarget(_objects[i]);
		OnBnClickedQinit();
		do{
			if (token == "r"){
				OnBnClickedGraspplanning();
			}
			else{
				break;
			}
		} while (cin >> token);
		OnBnClickedPathplanning();
		OnBnClickedPathsmoothing();
		//OnBnClickedQinit();
		//OnBnClickedButton8();
		//OnBnClickedQgoal();
		//OnBnClickedPathplanning();
		//OnBnClickedPathsmoothing();
		OnBnClickedQinit();
		OnBnClickedButton10();
		OnBnClickedButton6();
		OnBnClickedQgoal();
		OnBnClickedPathplanning();
		OnBnClickedPathsmoothing();
		OnBnClickedQinit();
		OnBnClickedButton8();
		OnBnClickedQgoal();
		OnBnClickedPathplanning();
		OnBnClickedPathsmoothing();
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton15()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	try{
		_kinect0 = new Kinect();
		_kinect1 = new Kinect(1);
		_kinect0->start();
		_kinect1->start();
		cout << "Connect OK!" << endl;
		(CButton*)GetDlgItem(IDC_BUTTON15)->EnableWindow(false);
	}
	catch (...){
		delete _kinect0;
		delete _kinect1;
		_kinect0 = 0;
		_kinect1 = 0;
		cout << "Connect Fault!" << endl;
	}
}

void CRobotSimulatorGUIDlg::OnBnClickedButton16()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_thread){
		_thread->join();
		delete _thread;
	}
	_thread = new thread([=]{foo1(_kinect0); });
}



void CRobotSimulatorGUIDlg::OnBnClickedButton17()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_thread){
		_thread->join();
		delete _thread;
	}
	_thread = new thread([=]{foo1(_kinect1); });
}


void CRobotSimulatorGUIDlg::OnBnClickedButton18()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	//if (_thread){
	//	_thread->join();
	//	delete _thread;
	//}
	//// set TransMat
	//_kinect1->setTransMat(Matrix4d::Identity());
	//vector<double> leafSize{ 0.005, 0.005, 0.005 };
	//PointCloud<PointXYZ>::Ptr kinectCloud0(new PointCloud<PointXYZ>), kinectCloud1(new PointCloud<PointXYZ>);
	//kinectCloud0 = downSampling(_kinect0->getPointCloud(), leafSize);
	//kinectCloud1 = downSampling(_kinect1->getPointCloud(), leafSize);
	//io::savePLYFile("kinect0.ply", *kinectCloud0);
	//io::savePLYFile("kinect1.ply", *kinectCloud1);
	//_thread = new thread([=]{foo(_transMat, kinectCloud0, kinectCloud1); });
	if (_pDlg == nullptr){
		_pDlg = new Calibration(_kinect0, _kinect1, &_transMat);
		_pDlg->Create(IDD_DIALOG1, this);
	}
	_pDlg->ShowWindow(SW_NORMAL);
}



void CRobotSimulatorGUIDlg::OnBnClickedButton19()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	// set TransMat
	_kinect1->setTransMat(_transMat);
	// save cloud
	_cloud = PointCloud<PointT>::Ptr(new PointCloud<PointT>());
	*_cloud = *_kinect0->getPointCloud() + *_kinect1->getPointCloud();
	io::savePLYFile("RawData/raw_data.ply", *_cloud);

	_cloud = PointCloud<PointT>::Ptr(new PointCloud<PointT>());
	io::loadPLYFile("raw_data.ply", *_cloud);

	////過濾副檔名
	//CString szFilter = CString("Polygon File Format|*.ply|All File|*.*||");
	////以Dialog方式開啟檔案
	//CFileDialog fd(true, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
	//if (fd.DoModal() == IDOK)  //按下OK 
	//{
	//	//開啟檔案成功
	//	CString szFileName = fd.GetPathName(); //取得開啟檔案的全名(包含路徑)
	//	_cloud = PointCloud<PointT>::Ptr(new PointCloud<PointT>());
	//	io::loadPLYFile(string(CT2A(szFileName)), *_cloud);
	//}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton20()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_thread){
		_thread->join();
		delete _thread;
	}
	_thread = new thread([=]{foo2(_cloud); });
}


void CRobotSimulatorGUIDlg::OnBnClickedButton21()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	vector<double> leafSize{ 0.005, 0.005, 0.005 };
	_cloud = downSampling(_cloud, leafSize);
}


void CRobotSimulatorGUIDlg::OnBnClickedButton22()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	CString cstring;
	string token;
	GetDlgItemTextW(IDC_EDIT4, cstring);
	token = CT2A(cstring);
	_xlimits[0] = stod(token);
	GetDlgItemTextW(IDC_EDIT6, cstring);
	token = CT2A(cstring);
	_ylimits[0] = stod(token);
	GetDlgItemTextW(IDC_EDIT8, cstring);
	token = CT2A(cstring);
	_zlimits[0] = stod(token);
	GetDlgItemTextW(IDC_EDIT5, cstring);
	token = CT2A(cstring);
	_xlimits[1] = stod(token);
	GetDlgItemTextW(IDC_EDIT7, cstring);
	token = CT2A(cstring);
	_ylimits[1] = stod(token);
	GetDlgItemTextW(IDC_EDIT9, cstring);
	token = CT2A(cstring);
	_zlimits[1] = stod(token);

	PointCloud<PointXYZ>::Ptr tempCloud;
	tempCloud = passThrough(_cloud, "x", _xlimits);
	tempCloud = passThrough(tempCloud, "y", _ylimits);
	tempCloud = passThrough(tempCloud, "z", _zlimits);
	_cloud = tempCloud;
}


void CRobotSimulatorGUIDlg::OnBnClickedButton23()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	PointIndices::Ptr inliers(new PointIndices);
	ModelCoefficients::Ptr coefficients(new ModelCoefficients);
	if (searchPlane(_cloud, inliers, coefficients)){
		_cloud = stayObjects(_cloud, inliers, coefficients);
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton24()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	segmentation(_cloud, _pclObjects);
	for (auto& i : _pclObjects){
		for (auto& j : *i){
			j.data[0] *= 1000;
			j.data[1] *= 1000;
			j.data[2] *= 1000;
		}
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton25()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	if (_thread){
		_thread->join();
		delete _thread;
	}
	_thread = new thread([=]{foo3(_pclObjects); });
}


void CRobotSimulatorGUIDlg::OnBnClickedButton26()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_normals.clear();
	_centroid.clear();
	for (auto& it : _pclObjects){
		_centroid.push_back(Vector4d());
		compute3DCentroid(*it, _centroid.back());
		_normals.push_back(PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>));
		_normals.back() = smoothingNormal(it, 30);
	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton27()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	_meshes.clear();
	for (auto& it : _normals){
		_meshes.push_back(PolygonMesh::Ptr(new PolygonMesh));
		_meshes.back() = createMesh(it, 30);
	}
	gMutex.lock();
	ostringstream os;
	for (int i = 0, n = _meshes.size(); i < n; ++i){
		os << i;
		string filename = "output/" + os.str() + ".obj";
		io::saveOBJFile(filename, *_meshes[i]);
		_objects.push_back(_demo->CreateObject(_demo->loadObjFile(&filename[0]), btVector3(1.0, 1.0, 1.0), Matrix4d::Identity(), COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		Matrix4d T = Matrix4d::Identity();
		T.block<4, 1>(0, 3) = _centroid[i];
		T = _T2base*T.eval();
		_objects[i]->setCOM6D(T);
		m_cbObject.AddString(CString(os.str().c_str()));
		os.str("");
	}
	gMutex.unlock();
}


void CRobotSimulatorGUIDlg::OnBnClickedButton30()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	fstream fout;
	fout.open("parameter/KinectTransMat", ios::out);
	fout << _transMat;
	fout.close();
	fout.open("parameter/TransMat2Base", ios::out);
	fout << _T2base;
	fout.close();
	fout.open("parameter/PathThroughParameters", ios::out);
	fout << _xlimits[0] << " " << _xlimits[1] << endl;
	fout << _ylimits[0] << " " << _ylimits[1] << endl;
	fout << _zlimits[0] << " " << _zlimits[1] << endl;
	fout.close();
	_simulatorThread.join();
	CDialogEx::OnOK();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit4()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit5()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit6()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit7()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit8()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeEdit9()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnCbnSelchangeCombo3()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	Matrix4d* Tptr = nullptr;
	VectorXd p;
	switch (index){
	case 0:
		Tptr = &_transMat;
		break;
	case 1:
		Tptr = &_T2base;
		break;
	}
	rbt::transMat2Vec(*Tptr, p);
	ostringstream os;
	os << p[0];
	SetDlgItemTextW(IDC_TRANSX, CString(os.str().c_str()));
	os.str("");
	os << p[1];
	SetDlgItemTextW(IDC_TRANSY, CString(os.str().c_str()));
	os.str("");
	os << p[2];
	SetDlgItemTextW(IDC_TRANSZ, CString(os.str().c_str()));
	os.str("");
	os << p[3] * 180 / M_PI;
	SetDlgItemTextW(IDC_TRANSRX, CString(os.str().c_str()));
	os.str("");
	os << p[4] * 180 / M_PI;
	SetDlgItemTextW(IDC_TRANSRY, CString(os.str().c_str()));
	os.str("");
	os << p[5] * 180 / M_PI;
	SetDlgItemTextW(IDC_TRANSRZ, CString(os.str().c_str()));
	os.str("");
}


void CRobotSimulatorGUIDlg::OnEnChangeTransx()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_TRANSX, cstring);
		try{
			(*Tptr)(0, 3) = stod(string(CT2A(cstring)));
		}
		catch (...){
			(*Tptr)(0, 3) = 0;
		}
		if (index == 1){
			lock_guard<mutex> mLock(gMutex);
			Matrix4d T;
			for (int i = 0, n = _meshes.size(); i < n; ++i){
				T = Matrix4d::Identity();
				T.block<4, 1>(0, 3) = _centroid[i];
				T = _T2base*T.eval();
				_objects[i]->setCOM6D(T);
			}
		}
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeTransy()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_TRANSY, cstring);
		try{
			(*Tptr)(1, 3) = stod(string(CT2A(cstring)));
		}
		catch (...){
			(*Tptr)(1, 3) = 0;
		}

		if (index == 1){
			lock_guard<mutex> mLock(gMutex);
			Matrix4d T;
			for (int i = 0, n = _meshes.size(); i < n; ++i){
				T = Matrix4d::Identity();
				T.block<4, 1>(0, 3) = _centroid[i];
				T = _T2base*T.eval();
				_objects[i]->setCOM6D(T);
			}
		}
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeTransz()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_TRANSZ, cstring);
		try{
			(*Tptr)(2, 3) = stod(string(CT2A(cstring)));
		}
		catch (...){
			(*Tptr)(2, 3) = 0;
		}

		if (index == 1){
			lock_guard<mutex> mLock(gMutex);
			Matrix4d T;
			for (int i = 0, n = _meshes.size(); i < n; ++i){
				T = Matrix4d::Identity();
				T.block<4, 1>(0, 3) = _centroid[i];
				T = _T2base*T.eval();
				_objects[i]->setCOM6D(T);
			}
		}
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeTransrx()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_TRANSRX, cstring);
		VectorXd p;
		rbt::transMat2Vec(*Tptr, p);
		Vector3d o;
		o = p.tail(3);
		try{
			o[0] = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			o[0] = 0;
		}
		Matrix3d R;
		kaxisTh2Rot(o, R);
		Tptr->block<3, 3>(0, 0) = R;

		if (index == 1){
			lock_guard<mutex> mLock(gMutex);
			Matrix4d T;
			for (int i = 0, n = _meshes.size(); i < n; ++i){
				T = Matrix4d::Identity();
				T.block<4, 1>(0, 3) = _centroid[i];
				T = _T2base*T.eval();
				_objects[i]->setCOM6D(T);
			}
		}
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeTransry()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_TRANSRY, cstring);
		VectorXd p;
		rbt::transMat2Vec(*Tptr, p);
		Vector3d o;
		o = p.tail(3);
		try{
			o[1] = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			o[1] = 0;
		}
		Matrix3d R;
		kaxisTh2Rot(o, R);
		Tptr->block<3, 3>(0, 0) = R;

		if (index == 1){
			lock_guard<mutex> mLock(gMutex);
			Matrix4d T;
			for (int i = 0, n = _meshes.size(); i < n; ++i){
				T = Matrix4d::Identity();
				T.block<4, 1>(0, 3) = _centroid[i];
				T = _T2base*T.eval();
				_objects[i]->setCOM6D(T);
			}
		}
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnEnChangeTransrz()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_TRANSRZ, cstring);
		VectorXd p;
		rbt::transMat2Vec(*Tptr, p);
		Vector3d o;
		o = p.tail(3);
		try{
			o[2] = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			o[2] = 0;
		}
		Matrix3d R;
		kaxisTh2Rot(o, R);
		Tptr->block<3, 3>(0, 0) = R;

		if (index == 1){
			lock_guard<mutex> mLock(gMutex);
			Matrix4d T;
			for (int i = 0, n = _meshes.size(); i < n; ++i){
				T = Matrix4d::Identity();
				T.block<4, 1>(0, 3) = _centroid[i];
				T = _T2base*T.eval();
				_objects[i]->setCOM6D(T);
			}
		}
	}
	UpdateData();
}


void CRobotSimulatorGUIDlg::OnBnClickedButton28()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	//過濾副檔名
	CString szFilter = CString("Text File|*.txt|All File|*.*||");
	//以Dialog方式開啟檔案
	CFileDialog fd(false, NULL, NULL, OFN_HIDEREADONLY, szFilter, NULL);
	if (fd.DoModal() == IDOK)  //按下OK 
	{
		string fileName = string(CT2A(fd.GetPathName()));
		if (fileName.compare(fileName.size() - 4, 4, ".txt") == 0){

		}
		else{
			fileName += ".txt";
		}
		fstream fout;
		fout.open(fileName.c_str(), ios::out);

		for (int i = 0, n = _paths.size() / 3; i < n; ++i){
			fout << "c" << endl;
			for (int j = 0, m = _paths[3 * i]->getPath().size() - 1; j < m; ++j){
				VectorXd q0 = _paths[3 * i]->getPath()[j], q1 = _paths[3 * i]->getPath()[j + 1];
				vector<VectorXd> intePath;
				interpolation(6, q0, q1, intePath);
				for (auto& it : intePath){
					fout << it.transpose() << endl;
				}
			}
			fout << "p" << endl;
			for (int j = 0, m = _paths[3 * i + 1]->getPath().size() - 1; j < m; ++j){
				VectorXd q0 = _paths[3 * i + 1]->getPath()[j], q1 = _paths[3 * i + 1]->getPath()[j + 1];
				vector<VectorXd> intePath;
				interpolation(6, q0, q1, intePath);
				for (auto& it : intePath){
					fout << it.transpose() << endl;
				}
			}
			fout << "r" << endl;
			for (int j = 0, m = _paths[3 * i + 2]->getPath().size() - 1; j < m; ++j){
				VectorXd q0 = _paths[3 * i + 2]->getPath()[j], q1 = _paths[3 * i + 2]->getPath()[j + 1];
				vector<VectorXd> intePath;
				interpolation(6, q0, q1, intePath);
				for (auto& it : intePath){
					fout << it.transpose() << endl;
				}
			}
			fout << "h" << endl;
			for (int j = 0, m = _paths[3 * i]->getQgoal().getHandTh().size(); j < m; ++j){
				if (j == 3){
					continue;
				}
				fout << _paths[3 * i]->getQgoal().getHandTh()[j] << " ";
			}
			fout << endl;
		}

		fout.close();
	}

	//bool* breakFlag = new bool(false);
	//thread recordThread = thread([=]{recordPath(_demo, breakFlag); });
	//OnBnClickedButton13();
	//*breakFlag = true;
	//recordThread.join();
	//delete breakFlag;
}


void CRobotSimulatorGUIDlg::OnBnClickedButton29()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_EDIT10, cstring);
		double angle = 0.0;
		try{
			angle = stod(string(CT2A(cstring)))*M_PI/180;
		}
		catch (...){
			angle = 0.0;
		}
		Matrix3d R;
		R = AngleAxisd(angle, Vector3d::UnitX());
		Tptr->block<3, 3>(0, 0) = R*Tptr->block<3, 3>(0, 0).eval();
		VectorXd p;
		rbt::transMat2Vec(*Tptr, p);
		ostringstream os;
		os << p[0];
		SetDlgItemTextW(IDC_TRANSX, CString(os.str().c_str()));
		os.str("");
		os << p[1];
		SetDlgItemTextW(IDC_TRANSY, CString(os.str().c_str()));
		os.str("");
		os << p[2];
		SetDlgItemTextW(IDC_TRANSZ, CString(os.str().c_str()));
		os.str("");
		os << p[3] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRX, CString(os.str().c_str()));
		os.str("");
		os << p[4] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRY, CString(os.str().c_str()));
		os.str("");
		os << p[5] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRZ, CString(os.str().c_str()));
		os.str("");

	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton31()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_EDIT10, cstring);
		double angle = 0.0;
		try{
			angle = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			angle = 0.0;
		}
		Matrix3d R;
		R = AngleAxisd(angle, Vector3d::UnitY());
		Tptr->block<3, 3>(0, 0) = R*Tptr->block<3, 3>(0, 0).eval();
		VectorXd p;
		rbt::transMat2Vec(*Tptr, p);
		ostringstream os;
		os << p[0];
		SetDlgItemTextW(IDC_TRANSX, CString(os.str().c_str()));
		os.str("");
		os << p[1];
		SetDlgItemTextW(IDC_TRANSY, CString(os.str().c_str()));
		os.str("");
		os << p[2];
		SetDlgItemTextW(IDC_TRANSZ, CString(os.str().c_str()));
		os.str("");
		os << p[3] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRX, CString(os.str().c_str()));
		os.str("");
		os << p[4] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRY, CString(os.str().c_str()));
		os.str("");
		os << p[5] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRZ, CString(os.str().c_str()));
		os.str("");

	}
}


void CRobotSimulatorGUIDlg::OnBnClickedButton32()
{
	// TODO:  在此加入控制項告知處理常式程式碼
	int index = m_cbTransMat.GetCurSel();
	if (index >= 0){
		Matrix4d* Tptr = nullptr;
		switch (index){
		case 0:
			Tptr = &_transMat;
			break;
		case 1:
			Tptr = &_T2base;
			break;
		}
		CString cstring;
		GetDlgItemTextW(IDC_EDIT10, cstring);
		double angle = 0.0;
		try{
			angle = stod(string(CT2A(cstring)))*M_PI / 180;
		}
		catch (...){
			angle = 0.0;
		}
		Matrix3d R;
		R = AngleAxisd(angle, Vector3d::UnitZ());
		Tptr->block<3, 3>(0, 0) = R*Tptr->block<3, 3>(0, 0).eval();
		VectorXd p;
		rbt::transMat2Vec(*Tptr, p);
		ostringstream os;
		os << p[0];
		SetDlgItemTextW(IDC_TRANSX, CString(os.str().c_str()));
		os.str("");
		os << p[1];
		SetDlgItemTextW(IDC_TRANSY, CString(os.str().c_str()));
		os.str("");
		os << p[2];
		SetDlgItemTextW(IDC_TRANSZ, CString(os.str().c_str()));
		os.str("");
		os << p[3] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRX, CString(os.str().c_str()));
		os.str("");
		os << p[4] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRY, CString(os.str().c_str()));
		os.str("");
		os << p[5] * 180 / M_PI;
		SetDlgItemTextW(IDC_TRANSRZ, CString(os.str().c_str()));
		os.str("");

	}
}

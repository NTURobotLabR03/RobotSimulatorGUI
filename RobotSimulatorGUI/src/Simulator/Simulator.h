/*
//類別名稱：Simulator
//作者：Chris Dickinson, Liu Yi-Ren
//日期：2016/08/15
//目的：將Robot類別, openGL(callback function), Bullet的類別整合在一個類別裡面
//使用函式庫：FreeGLUT, Bullet, Eigen
*/
#pragma once

#include "BulletHeader.h"
#include "EigenHeader.h"
#include "FreeGlutHeader.h"
#include "DebugDrawer.h"
#include "GlDrawcallback.h"
#include "Object.h"
#include "robot.h"
#include "RobotArm.h"
#include "RobotHand.h"
#include <vector>
using namespace rbt;

// 用來分類不同物體的 enum
// enum不懂不要問我 自己查
// 至於這種格式你參考bullet collision group
#define BIT(x) (1<<(x))
enum CollisionGroups{
	COLLISIONGROUP_NOTHING = 0,
	COLLISIONGROUP_ARM = BIT(0),
	COLLISIONGROUP_HANDBASE = BIT(1),
	COLLISIONGROUP_FINGER = BIT(2),
	// 大拇指較為特殊
	COLLISIONGROUP_THUMB = BIT(3),
	COLLISIONGROUP_ENV = BIT(4)
};

class Simulator
{
public:
	// 建構
	Simulator();
	// 解構
	virtual ~Simulator();
	// FreeGlut Callbacks
	// 鍵盤事件
	virtual void Keyboard(unsigned char key, int x, int y);
	// 鍵盤提起事件
	virtual void KeyboardUp(unsigned char key, int x, int y);
	// Special key鍵盤事件 Special Key對應的macro請查ppt
	virtual void Special(int key, int x, int y);
	// Special key鍵盤提起事件
	virtual void SpecialUp(int key, int x, int y);
	// 視窗縮放事件
	virtual void Reshape(int w, int h);
	// 沒事就會進入事件
	virtual void Idle();
	// 滑鼠事件
	virtual void Mouse(int button, int state, int x, int y);
	// 不常用
	virtual void PassiveMotion(int x, int y);
	// 滑鼠移動事件
	virtual void Motion(int x, int y);
	// 沒用到
	virtual void Display();

	// GL init
	void Initialize();

	// GL drawing functions
	// GL畫Box
	void DrawBox(const btVector3 &halfSize);
	// GL畫球
	void DrawSphere(const btScalar &radius);
	// GL畫圓柱
	void DrawCylinder(const btScalar &radius, const btScalar &halfHeight);
	// GL畫convex hull
	void DrawConvexHull(const btCollisionShape* shape);
	// GL畫各種bullet形狀的物體
	void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color);

	// GL camera function
	void UpdateCamera();

	// physics functions. Can be overrideen by derived classes (like BasicDemo)
	// init bullet環境
	virtual void InitializeBullet();
	// 關閉 bullet 環境
	virtual void ShutdownBullet();

	// collision event functions
	// 給手掌碰撞偵測用
	virtual void CheckForCollisionEvents();

	// draw object function
	// 幫忙把所有object畫在openGL畫布上的function
	void RenderScene();

	// object functions
	// 讀相對入徑或絕對入徑的obj檔 (切記不能有中文) 回傳btCollisionShape的指標 配合下面函式可以方便新增一個object
	btCollisionShape* loadObjFile(char* filename);
	// create object的function
	Object* CreateObject(btCollisionShape* pShape, const btVector3 &color = btVector3(1.0f, 1.0f, 1.0f), const Eigen::Matrix4d &transMat = Eigen::Matrix4d::Identity(), short int group = -1, short int mask = -1);

	// robot env function
	virtual void InitRobotEnv();
protected:
	// GL camera
	int old_rot_x = 0;
	int old_rot_y = 0;
	float distance = 0;
	int old_dis_y = 0;
	int rot_x = 0;
	int rot_y = 0;
	int record_x = 0;
	int record_y = 0;
	int currentButton = GLUT_TOT_BUTTON;
	float trans_x = 0;
	float trans_y = 0;
	int old_tran_x = 0;
	int old_tran_y = 0;
	int m_screenWidth;
	int m_screenHeight;
	float m_nearPlane = 1;
	float m_farPlane = 100000;

	// some world infomation
	btVector3 aabbMin;
	btVector3 aabbMax;

	// core Bullet components
	btBroadphaseInterface* m_pBroadphase = 0;
	btCollisionConfiguration* m_pCollisionConfiguration = 0;
	btCollisionDispatcher* m_pDispatcher = 0;
	btCollisionWorld* m_pWorld = 0;

	// debug renderer
	DebugDrawer* m_pDebugDrawer = 0;

	// my robot
	RobotArm* _arm = 0;
	RobotHand* _hand = 0;

	// an array of all objects
	std::vector<Object*> m_objects;

	// min contact error dist
	const double _contactEps = 0.1;
};
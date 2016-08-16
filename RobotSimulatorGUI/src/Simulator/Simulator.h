/*
//���O�W�١GSimulator
//�@�̡GChris Dickinson, Liu Yi-Ren
//����G2016/08/15
//�ت��G�NRobot���O, openGL(callback function), Bullet�����O��X�b�@�����O�̭�
//�ϥΨ禡�w�GFreeGLUT, Bullet, Eigen
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

// �ΨӤ������P���骺 enum
// enum�������n�ݧ� �ۤv�d
// �ܩ�o�خ榡�A�Ѧ�bullet collision group
#define BIT(x) (1<<(x))
enum CollisionGroups{
	COLLISIONGROUP_NOTHING = 0,
	COLLISIONGROUP_ARM = BIT(0),
	COLLISIONGROUP_HANDBASE = BIT(1),
	COLLISIONGROUP_FINGER = BIT(2),
	// �j��������S��
	COLLISIONGROUP_THUMB = BIT(3),
	COLLISIONGROUP_ENV = BIT(4)
};

class Simulator
{
public:
	// �غc
	Simulator();
	// �Ѻc
	virtual ~Simulator();
	// FreeGlut Callbacks
	// ��L�ƥ�
	virtual void Keyboard(unsigned char key, int x, int y);
	// ��L���_�ƥ�
	virtual void KeyboardUp(unsigned char key, int x, int y);
	// Special key��L�ƥ� Special Key������macro�Ьdppt
	virtual void Special(int key, int x, int y);
	// Special key��L���_�ƥ�
	virtual void SpecialUp(int key, int x, int y);
	// �����Y��ƥ�
	virtual void Reshape(int w, int h);
	// �S�ƴN�|�i�J�ƥ�
	virtual void Idle();
	// �ƹ��ƥ�
	virtual void Mouse(int button, int state, int x, int y);
	// ���`��
	virtual void PassiveMotion(int x, int y);
	// �ƹ����ʨƥ�
	virtual void Motion(int x, int y);
	// �S�Ψ�
	virtual void Display();

	// GL init
	void Initialize();

	// GL drawing functions
	// GL�eBox
	void DrawBox(const btVector3 &halfSize);
	// GL�e�y
	void DrawSphere(const btScalar &radius);
	// GL�e��W
	void DrawCylinder(const btScalar &radius, const btScalar &halfHeight);
	// GL�econvex hull
	void DrawConvexHull(const btCollisionShape* shape);
	// GL�e�U��bullet�Ϊ�������
	void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color);

	// GL camera function
	void UpdateCamera();

	// physics functions. Can be overrideen by derived classes (like BasicDemo)
	// init bullet����
	virtual void InitializeBullet();
	// ���� bullet ����
	virtual void ShutdownBullet();

	// collision event functions
	// ����x�I��������
	virtual void CheckForCollisionEvents();

	// draw object function
	// ������Ҧ�object�e�bopenGL�e���W��function
	void RenderScene();

	// object functions
	// Ū�۹�J�|�ε���J�|��obj�� (���O���঳����) �^��btCollisionShape������ �t�X�U���禡�i�H��K�s�W�@��object
	btCollisionShape* loadObjFile(char* filename);
	// create object��function
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
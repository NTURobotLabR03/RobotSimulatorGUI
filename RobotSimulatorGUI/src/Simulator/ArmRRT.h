/*
//���O�W�١GArmRRTManager(�~��RRTManager)
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�NRRTManager���I�������ޤJBullet���I������
//      �ΨӳW�����u�����|
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "BulletHeader.h"
#include "FreeGlutHeader.h"
#include "RobotArm.h"
#include "RobotHand.h"
#include "RRT.h"
using namespace rbt;

class ArmRRTManager: public RRTManager{
public:
	ArmRRTManager(btCollisionWorld* world = 0, RobotArm* arm = 0, RobotHand* hand  = 0);
	~ArmRRTManager();
	
	virtual bool RRTConnectPlanner(Tree* Ta, Tree* Tb, const Node& qinit, const Node& qgoal, std::vector<Eigen::VectorXd>& path) override;

	void setDebugDrawFlag(bool flag);
	bool getDebugDrawFlag();
	void draw();

protected:
	virtual void randomConfig(Node& qrand) override;
	virtual bool checkCollision(const Node& q) override;
	btCollisionWorld* _world;
	RobotArm* _arm;
	RobotHand* _hand;
	bool _debugDrawFlag = false;
	bool _firstTime = true;
};
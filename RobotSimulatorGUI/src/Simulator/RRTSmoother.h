/*
//���O�W�١GRRTSmoother
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�Q��Path Pruning Algorithm�h�ץ�RRT-Connect�X�Ӫ��y��
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "BulletHeader.h"
#include "FreeGlutHeader.h"
#include "RobotArm.h"
#include "RobotHand.h"
#include "trajPlan.h"
using namespace rbt;

class RRTSmoother{
public:
	RRTSmoother(btCollisionWorld* world = 0, RobotArm* arm = 0, RobotHand* hand = 0);
	~RRTSmoother();

	// ²����A �N���path��J(inputPath), ���Ƥƫ�|�s�J(outputPath) 
	void planning(const std::vector<Eigen::VectorXd>& inputPath, std::vector<Eigen::VectorXd>& outputPath);

private:
	bool checkCollision();
	btCollisionWorld* _world;
	RobotArm* _arm;
	RobotHand* _hand;
};
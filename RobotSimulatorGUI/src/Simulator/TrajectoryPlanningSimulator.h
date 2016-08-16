/*
//���O�W�١GTrajectoryPlanningSimulator(�~��GraspPlanningSimulator) // ���ӥs��PathPlanningSimulator����n
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�i�槹Grasp Planning�� �|�M�wFinal Configuration
//      ���FInitial Configuration �� Final Configuration��|�i��RRT-Connect Algorithm�i����|�W��
//      �Ӹ��|�W����y��|���_���ҥH�b�g�LPath Pruning Algorithm�i�業�Ƥ�
//�ϥΨ禡�w�GFreeGLUT, Bullet, Eigen, Qhull
*/
#pragma once
#include "GraspPlanningSimulator.h"
#include "ArmRRT.h"
#include "RRTSmoother.h"
#include "trajPlan.h"

class TrajectoryPlanningSimulator :public GraspPlanningSimulator{
public:
	TrajectoryPlanningSimulator();
	virtual ~TrajectoryPlanningSimulator();

	// FreeGlut Callbacks
	virtual void Keyboard(unsigned char key, int x, int y) override;
	virtual void Special(int key, int x, int y) override;
	virtual void Idle() override;

	// robot env function
	virtual void InitRobotEnv() override;

	// check normal collision ���y��W����
	bool checkCollision(); 
	//�ɥRCheckForCollisionEvents();�O��x��Ϊ�XD

protected:
	ArmRRTManager* _rrt = 0;
	RRTSmoother* _smoother = 0;
	bool _run = false;
	std::vector<Eigen::VectorXd> _path;
};
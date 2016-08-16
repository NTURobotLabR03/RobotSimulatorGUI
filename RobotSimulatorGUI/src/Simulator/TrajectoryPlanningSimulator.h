/*
//類別名稱：TrajectoryPlanningSimulator(繼承GraspPlanningSimulator) // 應該叫做PathPlanningSimulator比較好
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：進行完Grasp Planning後 會決定Final Configuration
//      有了Initial Configuration 跟 Final Configuration後會進行RRT-Connect Algorithm進行路徑規劃
//      而路徑規劃後軌跡會有震盪所以在經過Path Pruning Algorithm進行平滑化
//使用函式庫：FreeGLUT, Bullet, Eigen, Qhull
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

	// check normal collision 測軌跡規劃用
	bool checkCollision(); 
	//補充CheckForCollisionEvents();是手掌抓用的XD

protected:
	ArmRRTManager* _rrt = 0;
	RRTSmoother* _smoother = 0;
	bool _run = false;
	std::vector<Eigen::VectorXd> _path;
};
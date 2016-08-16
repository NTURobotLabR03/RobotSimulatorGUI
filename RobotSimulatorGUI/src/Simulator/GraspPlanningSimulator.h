/*
//類別名稱：GraspPlanningSimulator(繼承Simulator)
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：會針對Target Object的AABB
//      亂數決定手臂的End Effector
//      抓握後會計算Quality Measure
//使用函式庫：FreeGLUT, Bullet, Eigen, Qhull
*/
#pragma once
#include "ContactSimulator.h"
#include "GraspPoseGenerator.h"
#include "GraspGenerator.h"

class GraspPlanningSimulator :public Simulator{
public:
	GraspPlanningSimulator();
	virtual ~GraspPlanningSimulator();

	// FreeGlut Callbacks
	virtual void Keyboard(unsigned char key, int x, int y) override;
	virtual void Special(int key, int x, int y) override;
	virtual void Idle() override;

	// collision event functions
	virtual void CheckForCollisionEvents() override;

	// robot env function
	virtual void InitRobotEnv() override;
protected:
	Object* _table = 0, *_target = 0;
	ContactMgr _contactMgr;
	FrictionCone* _frictionCone = 0;
	WrenchGenerator* _wrenchGenerator = 0;
	GraspWrenchSpaceQualityMeasure* _qualityMeasure = 0;
	GraspPoseGenerator* _graspPoseGenerator = 0;
	GraspGenerator* _graspGenerator = 0;
};
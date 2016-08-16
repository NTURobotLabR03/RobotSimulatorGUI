/*
//類別名稱：ContactSimulator(繼承Simulator)
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：將所有碰撞資訊記錄起來
//      並且會針對手掌碰撞(contact filter)
//      計算GWS並且算出Quality Measure
//使用函式庫：FreeGLUT, Bullet, Eigen, Qhull
*/
#pragma once
#include "Simulator.h"
#include "ContactMgr.h"
#include "FrictionCone.h"
#include "WrenchGenerator.h"
#include "GraspWrenchSpaceQualityMeasure.h"

class ContactSimulator :public Simulator{
public:
	ContactSimulator();
	virtual ~ContactSimulator();
	// FreeGlut Callbacks
	virtual void Keyboard(unsigned char key, int x, int y) override;
	virtual void Special(int key, int x, int y) override;
	virtual void Idle() override;

	// collision event functions
	virtual void CheckForCollisionEvents() override;

	// robot env function
	virtual void InitRobotEnv() override;
protected:
	ContactMgr _contactMgr;
	FrictionCone _frictionCone;
	WrenchGenerator* _wrenchGenerator = 0;
	GraspWrenchSpaceQualityMeasure* _qualityMeasure = 0;
};
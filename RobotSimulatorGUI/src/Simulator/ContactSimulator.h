/*
//���O�W�١GContactSimulator(�~��Simulator)
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�N�Ҧ��I����T�O���_��
//      �åB�|�w���x�I��(contact filter)
//      �p��GWS�åB��XQuality Measure
//�ϥΨ禡�w�GFreeGLUT, Bullet, Eigen, Qhull
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
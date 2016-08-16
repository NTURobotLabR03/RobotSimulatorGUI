/*
//���O�W�١GGraspPlanningSimulator(�~��Simulator)
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�|�w��Target Object��AABB
//      �üƨM�w���u��End Effector
//      �촤��|�p��Quality Measure
//�ϥΨ禡�w�GFreeGLUT, Bullet, Eigen, Qhull
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
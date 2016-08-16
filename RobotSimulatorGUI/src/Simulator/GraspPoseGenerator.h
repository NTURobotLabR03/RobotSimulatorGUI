/*
//���O�W�١GGraspPoseGenerator
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�ΨӶüƲ��ͤ��u�촤��m����
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "BulletHeader.h"
#include "Object.h"
#include "RobotHand.h"
#include "RobotArm.h"
#include "rnGen.h"
using namespace rbt;

class GraspPoseGenerator{
public:
	// �b�غc�l�o��M�w Collision World, Arm, Hand, Target Object
	GraspPoseGenerator(btCollisionWorld* world = 0,RobotArm* arm = 0, RobotHand* hand = 0, Object* target = 0);
	~GraspPoseGenerator();
	// ��function�|�btarget object����üƧ��@�դ��uend effector�i�H�쪺�a��A �åB���ʹL�h
	// �p�G���鰪�ר����� �|����إͶüƥͦ����覡(downGrasp(), leftGrasp())
	void generate();

private:
	// �ѤW���U�����A
	void downGrasp();
	// �ѥ����k�����A
	void leftGrasp();
	// check �O�_���I��
	bool checkCollision();
	btCollisionWorld* _world;
	RobotArm* _arm;
	RobotHand* _hand;
	Object* _target;
};
/*
//���O�W�١GGraspGenerator
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�ΨӲ��ͤ�x�촤����(��x�}��X)
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "Object.h"
#include "RobotHand.h"
#include "ContactMgr.h"
#include "BulletHeader.h"
using namespace rbt;

class GraspGenerator{
public:
	// �b�غc�l�o��M�w Collision World, Arm, Hand, Target Object�H�� ContactMgr
	GraspGenerator(btCollisionWorld* world = 0, RobotHand* hand = 0, Object* target = 0, ContactMgr* contactMgr = 0, double contactEps = 0.1);
	~GraspGenerator();
	// �Ϥ�x�촤 ����I�� (���|���ʵe�X�{)
	void grasp();

private:
	// check �O�_�I�� ����Y�V
	void checkCollision();
	btCollisionWorld* _world;
	Object* _target;
	RobotHand* _hand;
	ContactMgr* _contactMgr;
	double _contactEps; // ���I����ǽT��
};
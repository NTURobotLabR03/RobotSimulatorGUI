/*
//類別名稱：GraspGenerator
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：用來產生手掌抓握的類(手掌開跟合)
//使用函式庫：Eigen, Bullet
*/
#pragma once
#include "Object.h"
#include "RobotHand.h"
#include "ContactMgr.h"
#include "BulletHeader.h"
using namespace rbt;

class GraspGenerator{
public:
	// 在建構子這邊決定 Collision World, Arm, Hand, Target Object以及 ContactMgr
	GraspGenerator(btCollisionWorld* world = 0, RobotHand* hand = 0, Object* target = 0, ContactMgr* contactMgr = 0, double contactEps = 0.1);
	~GraspGenerator();
	// 使手掌抓握 直到碰撞 (不會有動畫出現)
	void grasp();

private:
	// check 是否碰撞 比較嚴苛
	void checkCollision();
	btCollisionWorld* _world;
	Object* _target;
	RobotHand* _hand;
	ContactMgr* _contactMgr;
	double _contactEps; // 讓碰撞更準確用
};
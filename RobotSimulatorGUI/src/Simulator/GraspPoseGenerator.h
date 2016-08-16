/*
//類別名稱：GraspPoseGenerator
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：用來亂數產生手臂抓握位置的類
//使用函式庫：Eigen, Bullet
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
	// 在建構子這邊決定 Collision World, Arm, Hand, Target Object
	GraspPoseGenerator(btCollisionWorld* world = 0,RobotArm* arm = 0, RobotHand* hand = 0, Object* target = 0);
	~GraspPoseGenerator();
	// 此function會在target object附近亂數找到一組手臂end effector可以到的地方， 並且移動過去
	// 如果物體高度足夠高 會有兩種生亂數生成的方式(downGrasp(), leftGrasp())
	void generate();

private:
	// 由上往下的姿態
	void downGrasp();
	// 由左往右的姿態
	void leftGrasp();
	// check 是否有碰撞
	bool checkCollision();
	btCollisionWorld* _world;
	RobotArm* _arm;
	RobotHand* _hand;
	Object* _target;
};
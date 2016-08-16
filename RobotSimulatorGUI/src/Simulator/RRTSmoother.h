/*
//類別名稱：RRTSmoother
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：利用Path Pruning Algorithm去修正RRT-Connect出來的軌跡
//使用函式庫：Eigen, Bullet
*/
#pragma once
#include "BulletHeader.h"
#include "FreeGlutHeader.h"
#include "RobotArm.h"
#include "RobotHand.h"
#include "trajPlan.h"
using namespace rbt;

class RRTSmoother{
public:
	RRTSmoother(btCollisionWorld* world = 0, RobotArm* arm = 0, RobotHand* hand = 0);
	~RRTSmoother();

	// 簡單明瞭 將原來path丟入(inputPath), 平滑化後會存入(outputPath) 
	void planning(const std::vector<Eigen::VectorXd>& inputPath, std::vector<Eigen::VectorXd>& outputPath);

private:
	bool checkCollision();
	btCollisionWorld* _world;
	RobotArm* _arm;
	RobotHand* _hand;
};
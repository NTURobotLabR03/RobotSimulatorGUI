/*
//類別名稱：RobotHand(繼承Robot)
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：建立描述RobotHand的Class
//使用函式庫：Eigen, Bullet
*/
#pragma once
#include "robot.h"

namespace rbt{
	class RobotHand :public Robot{
	public:
		RobotHand();
		RobotHand(const Eigen::Matrix4d& T);
		~RobotHand();
		// 改變手掌在空間中位置
		void setBasePose(const Eigen::Matrix4d& T);
		// 連接到手臂上
		void connectToArm(Robot* arm);
		// 抓抓直到碰撞
		void grasp();
		// 打開手掌
		void release();
		// 設定要不要動 (_motionFlag true 的話會在IDLE慢慢移動) 備註UISimulator沒用到
		void setMotionFlag(bool flag);
		bool getMotionFlag();
		// 會建立每個finger對應的frame
		void setFingerRelation();
		// 將大拇指往內靠
		void closeThumbFinger();
		// reset robot
		virtual void reset() override;
	protected:
		bool _motionFlag = false;
		std::vector<std::vector<Frame*> > _finger;
	};
}
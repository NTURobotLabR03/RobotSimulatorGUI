/*
//類別名稱：RobotArm(繼承Robot)
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：建立描述RobotArm的Class
//使用函式庫：Eigen, Bullet
*/
#pragma once
#include "robot.h"

namespace rbt{
	class RobotArm:public Robot{
	public:
		// 解IK input簡化
		void dlsIK(const Eigen::Matrix4d& T, std::vector<double>& jointValue);
		// 計算end effector位置 output簡化
		void getFK(Eigen::Vector3d& endEffectorPos) const;
		// 計算end effector姿態 output簡化
		void getFK(Eigen::Matrix4d& endEffectorPos) const;
		// 給Eigen Vector也能改變手臂joint values
		void setTh(const Eigen::VectorXd& th);
		// 黏住物體Object
		void setStickObject(Object* stickObject);
		// 清除黏住物體的連結
		void clearStickObject();
		// reset robot
		virtual void reset() override;
		// 每當有frame轉動, 一定要call這個function 會幫忙更新Object的位置
		virtual void setObjectPose() override;
	protected:
		Object* _stickObject = nullptr;
		Eigen::Matrix4d _arm2Object;
	};
}
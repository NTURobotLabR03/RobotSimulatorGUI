/*
//���O�W�١GRobotArm(�~��Robot)
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�إߴy�zRobotArm��Class
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "robot.h"

namespace rbt{
	class RobotArm:public Robot{
	public:
		// ��IK input²��
		void dlsIK(const Eigen::Matrix4d& T, std::vector<double>& jointValue);
		// �p��end effector��m output²��
		void getFK(Eigen::Vector3d& endEffectorPos) const;
		// �p��end effector���A output²��
		void getFK(Eigen::Matrix4d& endEffectorPos) const;
		// ��Eigen Vector�]����ܤ��ujoint values
		void setTh(const Eigen::VectorXd& th);
		// �H����Object
		void setStickObject(Object* stickObject);
		// �M���H���骺�s��
		void clearStickObject();
		// reset robot
		virtual void reset() override;
		// �C��frame���, �@�w�ncall�o��function �|������sObject����m
		virtual void setObjectPose() override;
	protected:
		Object* _stickObject = nullptr;
		Eigen::Matrix4d _arm2Object;
	};
}
/*
//���O�W�١GRobotHand(�~��Robot)
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�إߴy�zRobotHand��Class
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "robot.h"

namespace rbt{
	class RobotHand :public Robot{
	public:
		RobotHand();
		RobotHand(const Eigen::Matrix4d& T);
		~RobotHand();
		// ���ܤ�x�b�Ŷ�����m
		void setBasePose(const Eigen::Matrix4d& T);
		// �s������u�W
		void connectToArm(Robot* arm);
		// ��쪽��I��
		void grasp();
		// ���}��x
		void release();
		// �]�w�n���n�� (_motionFlag true ���ܷ|�bIDLE�C�C����) �Ƶ�UISimulator�S�Ψ�
		void setMotionFlag(bool flag);
		bool getMotionFlag();
		// �|�إߨC��finger������frame
		void setFingerRelation();
		// �N�j��������a
		void closeThumbFinger();
		// reset robot
		virtual void reset() override;
	protected:
		bool _motionFlag = false;
		std::vector<std::vector<Frame*> > _finger;
	};
}
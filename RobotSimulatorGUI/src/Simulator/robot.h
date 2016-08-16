/*
//���O�W�١GRobot
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�إߴy�zRobot��Class
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "frame.h"
#include <Eigen/StdVector>

namespace Eigen{
	typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Matrix4dList;
	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dList;
}
namespace rbt{
	class Robot
	{
	public:
		// �غc�|��push_back�@��BasicFrame�b_allFrameList��_baseFrameList
		Robot();
		Robot(const Eigen::Matrix4d& T);
		~Robot();
		// �W�[���q����ʪ�Frame
		void addBasicFrame(const Eigen::Matrix4d& T, int parentId = -1);
		// �W�[�@�ӹ�ʪ��b
		void addRevoluteFrame(double a, double alpha, double d, double theta, double min, double max, int parentId = -1);
		// �W�[�@�ӳQ�ʪ��b
		void addPassiveFrame(double a, double alpha, double d, double theta, double ratio, int activeParentId, int parentId = -1);
		// DH�ئn�ncall�@���o��function
		void searchEE();
		// �e�δΥ�(openGL) �{�b�w�g���Ϊ�function
		void drawRobot() const;
		// �^�Ǧۥѫ׼ƶq
		unsigned DOFsize() const;
		// �^��End effector�ƶq
		unsigned EEsize() const;
		// operator [] �|�^�ǲ�i��Frame
		const DHFrame& operator[] (unsigned i) const;
		DHFrame& operator[] (unsigned i);
		// �p��End effectors����m
		void getFK(std::vector<Eigen::Vector3d>& endEffectorPos) const;
		// �p��End effectors�����A(position + orientation)
		void getFK(Eigen::Matrix4dList& endEffectorMat) const;
		// �o��Jacobian Matrix(�u��position������)
		void getJacobian(Eigen::MatrixXd& J) const;
		// �o��Jacobian Matrix
		void getJacobianWO(Eigen::MatrixXd& J) const;
		// reset robot
		virtual void reset();
		// ��IK (position) �Nfinal configuration�s�J jointValue
		void dlsIK(const std::vector<Eigen::Vector3d>& targetPos, std::vector<double>& jointValue);
		// ��IK �Nfinal configuration�s�J jointValue
		void dlsIK(const Eigen::Matrix4dList& targetPosMat, std::vector<double>& jointValue);

		// 20160421
		std::vector<Frame*>& getAllFrameList();
		// �C��frame���, �@�w�ncall�o��function �|������sObject����m
		virtual void setObjectPose();
	protected:
		std::vector<Frame*> _allFrameList, _baseFrameList;
		std::vector<DHFrame*> _jointFrameList;
		std::vector<PDHFrame*> _passiveFrameList;
		std::vector<EndEffector*> _endEffectorList;
	};

	void transMat2Vec(const Eigen::Matrix4d& transMat, Eigen::VectorXd& P);
}
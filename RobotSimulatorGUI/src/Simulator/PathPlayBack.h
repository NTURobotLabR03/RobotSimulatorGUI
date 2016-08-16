/*
//���O�W�١GPathPlayBack
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�ΨӦ^���������y��Ϊ�
//�ϥΨ禡�w�GEigen
*/
#pragma once
#include "EigenHeader.h"
#include "HandArmState.h"
#include "Object.h"

class PathPlayBack{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PathPlayBack();
	// �b�غc�l������l��̲�HandArmState�åB�O�������y��A�p�G���촤�F�� �]�wobject������ ������촤�� �������uend effector��transformation matrix
	PathPlayBack(const HandArmState& Qinit, const HandArmState& Qgoal, const std::vector<Eigen::VectorXd> path, Object* obj = 0, Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity());
	~PathPlayBack();
	// �d�߭y��
	const std::vector<Eigen::VectorXd>& getPath() const;
	// �d�ߪ�l���A
	const HandArmState& getQinit() const;
	// �d�̲߳ת��A
	const HandArmState& getQgoal() const;
	// �d��object�Pend effector��transformation matrix
	const Eigen::Matrix4d& getObjT0() const;
	// ����������骺����
	Object* getObj();

private:
	std::vector<Eigen::VectorXd> _path;
	HandArmState _Qinit, _Qgoal;
	Eigen::Matrix4d _T0;
	Object* _obj;
};
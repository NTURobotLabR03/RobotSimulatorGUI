#include "stdafx.h"
#include "RobotArm.h"
using namespace std;
using namespace rbt;
using namespace Eigen;

void RobotArm::dlsIK(const Eigen::Matrix4d& T, std::vector<double>& jointValue){
	Matrix4dList targetPosMat;
	targetPosMat.push_back(T);
	Robot::dlsIK(targetPosMat, jointValue);
}

void RobotArm::reset() {
	Robot::reset();
	for (int i = 0, n = _allFrameList.size(); i < n; ++i){
		if (_allFrameList[i]->getObject()){
			_allFrameList[i]->getObject()->setContact(false);
		}
	}
	setObjectPose();
}

void RobotArm::getFK(Vector3d& endEffectorPos) const{
	vector<Vector3d> temp;
	Robot::getFK(temp);
	endEffectorPos = temp[0];
}

void RobotArm::getFK(Eigen::Matrix4d& endEffectorPos) const{
	Matrix4dList temp;
	Robot::getFK(temp);
	endEffectorPos = temp[0];
}

void RobotArm::setTh(const Eigen::VectorXd& th){
	for (int i = 0, n = DOFsize(); i < n; ++i){
		(*this)[i].setCmd(th[i]);
	}
	setObjectPose();
}

void RobotArm::setStickObject(Object* stickObject){
	_stickObject = stickObject;
	Matrix4d base2Arm;
	getFK(base2Arm);
	_arm2Object = base2Arm.inverse() * _stickObject->getCOM6D();
}

void RobotArm::clearStickObject(){
	_stickObject = nullptr;
}

void RobotArm::setObjectPose(){
	Robot::setObjectPose();
	if (_stickObject){
		Matrix4d base2Arm;
		getFK(base2Arm);
		_stickObject->setCOM6D(base2Arm*_arm2Object);
	}
}
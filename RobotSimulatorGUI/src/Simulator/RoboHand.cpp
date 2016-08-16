#include "stdafx.h"
#include "RobotHand.h"
#include "ThreadControl.h"
using namespace std;
using namespace rbt;

rbt::RobotHand::RobotHand(){
	rbt::Robot::Robot();
}

rbt::RobotHand::RobotHand(const Eigen::Matrix4d& T){
	rbt::Robot::Robot(T);
}

rbt::RobotHand::~RobotHand(){
	rbt::Robot::~Robot();
}

void rbt::RobotHand::setBasePose(const Eigen::Matrix4d& T){
	((BasicFrame*)_allFrameList[0])->setTransMat(T);
}

void rbt::RobotHand::connectToArm(Robot* arm){
	Eigen::Matrix4dList armFK;
	arm->getFK(armFK);

	Eigen::Matrix4d T;
	// 把arm end effector 座標系轉成 手掌base 座標系
	T << 0, 0, 1, 0,
		0, 1, 0, 0,
		-1, 0, 0, 0,
		0, 0, 0, 1;
	T = armFK[0] * T;
	setBasePose(T);
	setObjectPose();
}

void rbt::RobotHand::setMotionFlag(bool flag){
	_motionFlag = flag;
}

void rbt::RobotHand::grasp(){
	static const double pi = acos(-1.0);
	static double cacheData[12] = { 0 };
	if (_motionFlag){
		_jointFrameList[0]->setCmd(pi / 2);
		for (int i = 0, n = _finger.size(); i < n; ++i){
			bool goFlag = true;
			for (int j = 0, m = _finger[i].size(); j < m; ++j){
				if (_finger[i][j]->getObject()->getContact()){
					goFlag = false;
					break;
				}
			}
			if (goFlag){
				for (int j = 0, m = _finger[i].size(); j < m; ++j){
					if (_finger[i][j]->isDrive()){
						((DHFrame*)_finger[i][j])->updateCmd(0.01);
					}
				}
			}
		}
	}
	bool checkSame = true;
	for (int i = 0; i < 12; ++i){
		if (_jointFrameList[i]->getCmd() != cacheData[i]){
			checkSame = false;
		}
		cacheData[i] = _jointFrameList[i]->getCmd();
	}
	if (checkSame)
		_motionFlag = false;

	setObjectPose();
}

void rbt::RobotHand::release(){
	const double pi = acos(-1.0);
	double cacheData[12] = { 0 };
	_motionFlag = true;
	for (int i = 0; i < 12; ++i){
		cacheData[i] = _jointFrameList[i]->getCmd();
	}
	while (_motionFlag){
		lock_guard<mutex> mLock(gMutex);
		this_thread::sleep_for(chrono::duration<int, milli>(1));
		for (int i = 1; i < 12; ++i){
			_jointFrameList[i]->updateCmd(-0.01);
		}
		setObjectPose();
		_motionFlag = false;
		// check
		for (int i = 0; i < 12; ++i){
			if (_jointFrameList[i]->getCmd() != cacheData[i]){
				_motionFlag = true;
				break;
			}
		}
		// update cacheData
		for (int i = 0; i < 12; ++i){
			cacheData[i] = _jointFrameList[i]->getCmd();
		}
	}
}

void rbt::RobotHand::setFingerRelation(){
	for (int i = 0; i < 5; ++i){
		_finger.push_back(vector<Frame*>());
	}
	_finger[0].push_back(_jointFrameList[1]);
	_finger[0].push_back(_jointFrameList[2]);
	_finger[0].push_back(_passiveFrameList[0]);

	_finger[1].push_back(_jointFrameList[4]);
	_finger[1].push_back(_jointFrameList[5]);
	_finger[1].push_back(_passiveFrameList[1]);

	_finger[2].push_back(_jointFrameList[6]);
	_finger[2].push_back(_jointFrameList[7]);
	_finger[2].push_back(_passiveFrameList[2]);

	_finger[3].push_back(_jointFrameList[8]);
	_finger[3].push_back(_jointFrameList[9]);
	_finger[3].push_back(_passiveFrameList[4]);

	_finger[4].push_back(_jointFrameList[10]);
	_finger[4].push_back(_jointFrameList[11]);
	_finger[4].push_back(_passiveFrameList[6]);
}

void rbt::RobotHand::reset() {
	Robot::reset();
	for (int i = 0, n = _allFrameList.size(); i < n; ++i){
		if (_allFrameList[i]->getObject()){
			_allFrameList[i]->getObject()->setContact(false);
		}
	}
	setObjectPose();
	_motionFlag = false;
}

bool rbt::RobotHand::getMotionFlag(){
	return _motionFlag;
}

void rbt::RobotHand::closeThumbFinger(){
	reset();
	(*this)[0].setCmd(M_PI / 2);
	setObjectPose();
}
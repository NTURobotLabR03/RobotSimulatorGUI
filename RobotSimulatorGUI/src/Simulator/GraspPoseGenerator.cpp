#include "stdafx.h"
#include "GraspPoseGenerator.h"
#include <vector>
using namespace std;
using namespace Eigen;

GraspPoseGenerator::GraspPoseGenerator(btCollisionWorld* world, RobotArm* arm, RobotHand* hand, Object* target){
	_world = world;
	_arm = arm;
	_hand = hand;
	_target = target;
}

GraspPoseGenerator::~GraspPoseGenerator(){}

void GraspPoseGenerator::downGrasp(){
	Vector3d aabbMin, aabbMax;
	Matrix3d R;
	Matrix4d T = Matrix4d::Identity();
	vector<double> jointValue;

	_hand->reset();
	(*_hand)[0].setCmd(M_PI / 2);
	_hand->setObjectPose();

	_target->getaabb(aabbMin, aabbMax);
	R = AngleAxisd(M_PI / 4, Vector3d::UnitY());
	T.block<3, 3>(0, 0) = R;
	T(0, 3) = rnGen(aabbMin[0], aabbMax[0]);
	T(1, 3) = rnGen(aabbMin[1] + 20, aabbMax[1] + 20);
	T(2, 3) = rnGen(aabbMax[2], aabbMax[2] + 115);
	_arm->dlsIK(T, jointValue);
	for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
		(*_arm)[i].setCmd(jointValue[i]);
	}
	_arm->setObjectPose();
	_hand->connectToArm(_arm);
	_hand->setObjectPose();
}

void GraspPoseGenerator::leftGrasp(){
	Vector3d aabbMin, aabbMax;
	Matrix3d R;
	Matrix4d T = Matrix4d::Identity();
	vector<double> jointValue;

	_hand->reset();
	(*_hand)[0].setCmd(M_PI / 2);
	_hand->setObjectPose();

	_target->getaabb(aabbMin, aabbMax);
	R = AngleAxisd(M_PI / 4, Vector3d::UnitY());
	T.block<3, 3>(0, 0) = R;
	R = AngleAxisd(-M_PI / 2, Vector3d::UnitX());
	T.block<3, 3>(0, 0) = R*T.block<3, 3>(0, 0).eval();
	T(0, 3) = rnGen(aabbMin[0], aabbMax[0]);
	T(1, 3) = rnGen(aabbMax[1], aabbMax[1] + 100);
	T(2, 3) = rnGen(aabbMin[2] + 40, aabbMax[2] + 10);
	_arm->dlsIK(T, jointValue);
	for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
		(*_arm)[i].setCmd(jointValue[i]);
	}
	_arm->setObjectPose();
	_hand->connectToArm(_arm);
	_hand->setObjectPose();
}

bool GraspPoseGenerator::checkCollision(){
	//Perform collision detection // 這行一定要有!
	_world->performDiscreteCollisionDetection();
	int numManifolds = _world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = _world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)contactManifold->getBody0();
		btCollisionObject* obB = (btCollisionObject*)contactManifold->getBody1();
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		if (contactManifold->getNumContacts()){
			return true;
		}
	}
	return false;
}

void GraspPoseGenerator::generate(){
	bool twoGraspFlag = false;
	Vector3d aabbMin, aabbMax;
	_target->getaabb(aabbMin, aabbMax);
	if (aabbMax[2] - aabbMin[2] > 100){
		twoGraspFlag = true;
	}

	if (twoGraspFlag){
		do{
			if (rnGen(2)){
				leftGrasp();
			}
			else{
				downGrasp();
			}
		} while (checkCollision());
	}
	else{
		do{
			downGrasp();
		} while (checkCollision());
	}
}
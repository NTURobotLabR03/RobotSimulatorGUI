#include "stdafx.h"
#include "ArmRRT.h"
#include "rnGen.h"
#include <Eigen/StdVector>
using namespace std;
using namespace Eigen;

ArmRRTManager::ArmRRTManager(btCollisionWorld* world, RobotArm* arm, RobotHand* hand){
	_world = world;
	_arm = arm;
	_hand = hand;
	_dim = arm->DOFsize();
}

ArmRRTManager::~ArmRRTManager(){

}

void ArmRRTManager::randomConfig(Node& qrand){
	double min = 0.0, max = 0.0;
	qrand.getData() = Eigen::VectorXd(_dim);
	for (int i = 0; i < _dim; ++i){
		(*_arm)[i].getCmdRange(min, max);
		qrand.getData()[i] = rnGen(min, max);
	}
}

bool ArmRRTManager::checkCollision(const Node& q){
	for (int i = 0; i < _dim; ++i){
		(*_arm)[i].setCmd(q.getData()[i]);
	}
	_arm->setObjectPose();
	_hand->connectToArm(_arm);
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

void ArmRRTManager::setDebugDrawFlag(bool flag){
	_debugDrawFlag = flag;
}

bool ArmRRTManager::getDebugDrawFlag(){
	return _debugDrawFlag;
}

void ArmRRTManager::draw(){

	static vector<Vector3d, aligned_allocator<Vector3d>> node;
	if (_debugDrawFlag){
		if (_firstTime){
			node.clear();
			for (auto& it : _vertices){
				for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
					(*_arm)[i].setCmd(it->getData()[i]);
				}
				node.push_back(Vector3d());
				_arm->getFK(node.back());
			}
			_firstTime = false;
		}
		glColor3f(0.4, 0.4, 0.4);
		for (auto& it : node){
			glPushMatrix();
			glTranslated(it[0], it[1], it[2]);
			glutSolidSphere(3, 4, 4);
			glPopMatrix();
		}
	}
}

bool ArmRRTManager::RRTConnectPlanner(Tree* Ta, Tree* Tb, const Node& qinit, const Node& qgoal, std::vector<Eigen::VectorXd>& path){
	_firstTime = true;
	return RRTManager::RRTConnectPlanner(Ta, Tb, qinit, qgoal, path);
}
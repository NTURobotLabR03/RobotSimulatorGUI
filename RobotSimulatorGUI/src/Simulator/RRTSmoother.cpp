#include "stdafx.h"
#include "RRTSmoother.h"
using namespace tp;
using namespace std;
using namespace Eigen;

RRTSmoother::RRTSmoother(btCollisionWorld* world, RobotArm* arm, RobotHand* hand){
	_world = world;
	_arm = arm;
	_hand = hand;
}

RRTSmoother::~RRTSmoother(){

}

void RRTSmoother::planning(const std::vector<Eigen::VectorXd>& inputPath, std::vector<Eigen::VectorXd>& outputPath){
	_hand->closeThumbFinger();
	vector<VectorXd> newPath;
	int j = inputPath.size() - 1;
	int segment = 0;
	vector<int> segments;
	while (j != 0){
		segments.push_back(segment);
		newPath.push_back(inputPath[j]);
		for (int i = 0; i < j; ++i){
			vector<VectorXd> viaPoints;
			bool collisionflag = false;
			segment = j - i;
			interpolation(segment , inputPath[i], inputPath[j], viaPoints);
			for (const auto& it : viaPoints){
				_arm->setTh(it);
				_hand->connectToArm(_arm);
				if (checkCollision()){
					collisionflag = true;
					break;
				}
			}
			if (!collisionflag){
				j = i;
				break;
			}
			if (i == j - 1){
				j = i;
				break;
			}
		}
	}
	segments.push_back(segment);
	newPath.push_back(inputPath[j]);
	outputPath.clear();
	for (int i = newPath.size() - 1; i > 0; --i){
		vector<VectorXd> viaPoints;
		interpolation(segments[i], newPath[i], newPath[i - 1], viaPoints);
		for (const auto& it : viaPoints){
			outputPath.push_back(it);
		}
	}
}

bool RRTSmoother::checkCollision(){
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
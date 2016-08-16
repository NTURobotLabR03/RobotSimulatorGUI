#include "stdafx.h"
#include "GraspGenerator.h"

GraspGenerator::GraspGenerator(btCollisionWorld* world, RobotHand* hand, Object* target, ContactMgr* contactMgr, double contactEps){
	_world = world;
	_hand = hand;
	_target = target;
	_contactMgr = contactMgr;
	_contactEps = contactEps;
}

GraspGenerator::~GraspGenerator(){}

void GraspGenerator::grasp(){
	_contactMgr->clear();
	_hand->reset();
	_hand->setMotionFlag(true);

	while(_hand->getMotionFlag()){
		_hand->grasp();
		checkCollision();
	}
}

void GraspGenerator::checkCollision(){
	//Perform collision detection // 這行一定要有!
	_world->performDiscreteCollisionDetection();
	int numManifolds = _world->getDispatcher()->getNumManifolds();
	//For each contact manifold
	_contactMgr->clear();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = _world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)contactManifold->getBody0();
		btCollisionObject* obB = (btCollisionObject*)contactManifold->getBody1();
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		int numContacts = contactManifold->getNumContacts();
		//For each contact point in that manifold
		for (int j = 0; j < numContacts; j++) {
			//Get the contact information
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			btVector3 ptB = pt.getPositionWorldOnB();
			double ptdist = pt.getDistance();
			// TODO
			if (ptdist < _contactEps){
				for (int i = 0, n = _hand->getAllFrameList().size(); i < n; ++i){
					if (_hand->getAllFrameList()[i]->getObject()){
						if (_hand->getAllFrameList()[i]->getObject()->getCollisionObject() == obA || _hand->getAllFrameList()[i]->getObject()->getCollisionObject() == obB){
							_hand->getAllFrameList()[i]->getObject()->setContact(true);
						}
					}
				}
				_contactMgr->push_back(new ContactInfo(CollisionPair(obA, obB), ptB, pt.m_normalWorldOnB, ptdist));
			}
		}
	}
}
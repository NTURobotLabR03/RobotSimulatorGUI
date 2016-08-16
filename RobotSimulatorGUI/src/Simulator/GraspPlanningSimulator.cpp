#include "stdafx.h"
#include "GraspPlanningSimulator.h"
#include "rnGen.h"
using namespace Eigen;

GraspPlanningSimulator::GraspPlanningSimulator(){}

GraspPlanningSimulator::~GraspPlanningSimulator(){
	_contactMgr.clear();
	delete _frictionCone;
	delete _wrenchGenerator;
	delete _qualityMeasure;
	delete _graspPoseGenerator;
	delete _graspGenerator;
}

void GraspPlanningSimulator::Keyboard(unsigned char key, int x, int y){

	vector<vector<double>> tt;
	switch (key){
	case 'w':
		m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawAabb);
		break;
	case 'f':
		_contactMgr.setDebugDrawFlag(!_contactMgr.getDebugDrawFlag());
		break;
	case 'q':
		if (_wrenchGenerator){
			delete _wrenchGenerator;
		}
		if (_qualityMeasure){
			delete _qualityMeasure;
		}
		_wrenchGenerator = new WrenchGenerator(_target, &_contactMgr, _frictionCone);
		_qualityMeasure = new GraspWrenchSpaceQualityMeasure();
		_wrenchGenerator->Union(tt);
		if (_qualityMeasure->compute(tt)){
			cout << _qualityMeasure->getEps() << endl;
			cout << _qualityMeasure->getVolume() << endl << endl;
		}
		break;
	case 'g':
		_graspGenerator->grasp();
		break;
	case 'p':
		_graspPoseGenerator->generate();
		break;
	case char(13) : // enter key
		_contactMgr.clear();
		_hand->reset();
		_hand->setMotionFlag(true);
		break;
	}
}

void GraspPlanningSimulator::Special(int key, int x, int y){

}

void GraspPlanningSimulator::Idle(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (_hand->getMotionFlag()){
		_hand->grasp();
		CheckForCollisionEvents();
	}

	UpdateCamera();

	_contactMgr.draw();

	RenderScene();

	glutSwapBuffers();
}

void GraspPlanningSimulator::CheckForCollisionEvents(){
	//Perform collision detection // 這行一定要有!
	m_pWorld->performDiscreteCollisionDetection();
	int numManifolds = m_pWorld->getDispatcher()->getNumManifolds();
	//For each contact manifold
	_contactMgr.clear();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = m_pWorld->getDispatcher()->getManifoldByIndexInternal(i);
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
				for (auto& it : m_objects)
				if (it->getCollisionObject() == obA || it->getCollisionObject() == obB)
					it->setContact(true);
				_contactMgr.push_back(new ContactInfo(CollisionPair(obA, obB), ptB, pt.m_normalWorldOnB, ptdist));
			}
		}
	}
}

void GraspPlanningSimulator::InitRobotEnv(){
	// init robot
	Simulator::InitRobotEnv();
	(*_hand)[0].setCmd(M_PI / 2);
	_hand->setObjectPose();
	// init env object
	Matrix4d T = Matrix4d::Identity();
	T(0, 3) = 830;
	T(2, 3) = -130;
	_table = CreateObject(new btBoxShape(btVector3(500, 350, 150)), btVector3(0.0, 0.6, 1.0), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	//T(0, 3) = 430;
	//T(1, 3) = 100;
	//T(2, 3) = 72;
	//_target = CreateObject(loadObjFile("model/Cup.obj"), btVector3(1.0f, 1.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	T(0, 3) = 530;
	T(1, 3) = 130;
	T(2, 3) = 89;
	_target = CreateObject(loadObjFile("model/bottle.obj"), btVector3(1.0f, 1.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	//T(0, 3) = 630;
	//T(1, 3) = 130;
	//T(2, 3) = 70;
	//_target = CreateObject(new btBoxShape(btVector3(30, 45, 50)), btVector3(1.0f, 1.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	//T(0, 3) = 630;
	//T(1, 3) = 130;
	//T(2, 3) = 52;
	//_target = CreateObject(loadObjFile("model/stick ps3.obj"), btVector3(0.0f, 0.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	//T(0, 3) = 530;
	//T(1, 3) = 130;
	//T(2, 3) = 50;
	//_target = CreateObject(new btSphereShape(30), btVector3(1.0f, 1.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	// init FrictionCone
	_frictionCone = new FrictionCone();
	// init GraspPoseGenerator
	_graspPoseGenerator = new GraspPoseGenerator(m_pWorld, _arm, _hand, _target);
	// init GraspGeneraor
	_graspGenerator = new GraspGenerator(m_pWorld, _hand, _target, &_contactMgr);
}
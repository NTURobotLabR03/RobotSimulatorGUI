#include "stdafx.h"
#include "ContactSimulator.h"
using namespace Eigen;

ContactSimulator::ContactSimulator(){}

ContactSimulator::~ContactSimulator(){}

void ContactSimulator::Idle(){
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

void ContactSimulator::Keyboard(unsigned char key, int x, int y){
	vector<vector<double>> tt;
	Matrix3d R;
	Matrix4d T;
	switch (key){
	case 'a':
		if (_wrenchGenerator){
			delete _wrenchGenerator;
		}
		if (_qualityMeasure){
			delete _qualityMeasure;
		}
		_wrenchGenerator = new WrenchGenerator(m_objects.back(), &_contactMgr, &_frictionCone);
		_qualityMeasure = new GraspWrenchSpaceQualityMeasure();
		_wrenchGenerator->Union(tt);
		if (_qualityMeasure->compute(tt)){
			cout << _qualityMeasure->getEps() << endl;
			cout << _qualityMeasure->getVolume() << endl << endl;
		}
		break;
	case 'p':
		for (int i = 0, n = _contactMgr.size(); i < n; ++i){
			if (m_objects.back()->getCollisionObject() == _contactMgr[i]->getCollisionPair().first || m_objects.back()->getCollisionObject() == _contactMgr[i]->getCollisionPair().second){
				cout << "Contact " << i << ":" << endl;
				cout << "Pair:" << (int)_contactMgr[i]->getCollisionPair().first << "," << (int)_contactMgr[i]->getCollisionPair().second << endl;
				cout << "Point:" << _contactMgr[i]->getPoint()[0] << " " << _contactMgr[i]->getPoint()[1] << " " << _contactMgr[i]->getPoint()[2] << endl;
				cout << "Normal:" << _contactMgr[i]->getNormal()[0] << " " << _contactMgr[i]->getNormal()[1] << " " << _contactMgr[i]->getNormal()[2] << endl;
				cout << "Distance:" << _contactMgr[i]->getDistance() << endl << endl;
			}
		}
		break;
	case 'z':
		R = AngleAxisd(0.02, Vector3d::UnitY());
		T = Matrix4d::Identity();
		T.block<3, 3>(0, 0) = R;
		m_objects.back()->setCOM6D(T*m_objects.back()->getCOM6D());
		break;
	case 'x':
		R = AngleAxisd(0.02, Vector3d::UnitX());
		T = Matrix4d::Identity();
		T.block<3, 3>(0, 0) = R;
		m_objects.back()->setCOM6D(T*m_objects.back()->getCOM6D());
		break;
	case char(13) : // enter key
		_contactMgr.clear();
		_hand->reset();
		_hand->setMotionFlag(true);
		break;
	case 's':
		_contactMgr.setDebugDrawFlag(!_contactMgr.getDebugDrawFlag());
		break;
	}
}

void ContactSimulator::Special(int key, int x, int y){
	Vector3d t;
	t = m_objects.back()->getCOM3D();
	switch (key){
	case GLUT_KEY_LEFT:
		t[1] += 1;
		break;
	case GLUT_KEY_RIGHT:
		t[1] -= 1;
		break;
	case GLUT_KEY_UP:
		t[0] += 1;
		break;
	case GLUT_KEY_DOWN:
		t[0] -= 1;
		break;
	case GLUT_KEY_PAGE_UP:
		t[2] += 1;
		break;
	case GLUT_KEY_PAGE_DOWN:
		t[2] -= 1;
		break;
	}
	m_objects.back()->setCOM3D(t);
}

void ContactSimulator::InitRobotEnv(){
	const double pi = acos(-1.0);
	_hand = new RobotHand();
	Matrix4d setRobotHandTrans;
	// 由於將原點移至手掌心
	// z減掉69
	// Thumb
	setRobotHandTrans << 1, 0, 0, 3.4,
		0, 1, 0, -22.38,
		0, 0, 1, -16.76,
		0, 0, 0, 1;
	_hand->addBasicFrame(setRobotHandTrans, 0); // id1
	_hand->addRevoluteFrame(50.74, pi / 2, 13.08, -pi / 2, 0, pi / 2, 1); // id2
	_hand->addRevoluteFrame(38.95, 0, 0, pi / 4, 0, pi / 2, 2); // id3
	_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 3); // id4
	_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 4, 4); // id5

	// Index
	setRobotHandTrans << 0, 0, 1, 12.05,
		1, 0, 0, -42.25,
		0, 1, 0, 11.59,
		0, 0, 0, 1;
	_hand->addBasicFrame(setRobotHandTrans, 0); // id6
	_hand->addRevoluteFrame(44.5, pi / 2, -9.65, pi / 2, 0, pi / 12, 6); // id7
	_hand->addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 7); // id8
	_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, pi * 99 / 180, 8); // id9
	_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 9, 9); // id10

	// Middle
	setRobotHandTrans << 0, 1, 0, 2.4,
		0, 0, 1, -17.4,
		1, 0, 0, 69.22,
		0, 0, 0, 1;
	_hand->addBasicFrame(setRobotHandTrans, 0); // id11
	_hand->addRevoluteFrame(38.88, 0, 0, 0, 0, pi / 2, 11); // id12
	_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, pi * 99 / 180, 12); // id13
	_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 13, 13); // id14

	// Ring
	setRobotHandTrans << 0, 0, -1, 12.05,
		-1, 0, 0, 5.75,
		0, 1, 0, 11.59,
		0, 0, 0, 1;
	_hand->addBasicFrame(setRobotHandTrans, 0); // id15
	_hand->addPassiveFrame(44.5, -pi / 2, 9.65, pi / 2, 1, 7, 15); // id16
	_hand->addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 16); // id17
	_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 17); // id18
	_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 18, 18); // id19

	// Pinky
	setRobotHandTrans << 0, 0, -1, 12.05,
		-1, 0, 0, 29.75,
		0, 1, 0, -1.39,
		0, 0, 0, 1;
	_hand->addBasicFrame(setRobotHandTrans, 0); // id20
	_hand->addPassiveFrame(44.5, -pi / 2, 9.65, pi / 2, 21.5 / 15, 7, 20); // id21
	_hand->addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 21); // id22
	_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 22); // id23
	_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 23, 23); // id24
	_hand->searchEE();

	// Base
	_hand->getAllFrameList()[0]->setObject(CreateObject(loadObjFile("model/hand/base_v2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_HANDBASE, COLLISIONGROUP_ENV));

	// Thumb
	_hand->getAllFrameList()[2]->setObject(CreateObject(loadObjFile("model/hand/thumb_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[3]->setObject(CreateObject(loadObjFile("model/hand/thumb_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[4]->setObject(CreateObject(loadObjFile("model/hand/thumb_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[5]->setObject(CreateObject(loadObjFile("model/hand/thumb_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));

	// Index
	_hand->getAllFrameList()[7]->setObject(CreateObject(loadObjFile("model/hand/index_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[8]->setObject(CreateObject(loadObjFile("model/hand/index_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[9]->setObject(CreateObject(loadObjFile("model/hand/index_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[10]->setObject(CreateObject(loadObjFile("model/hand/index_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));

	// middle
	_hand->getAllFrameList()[12]->setObject(CreateObject(loadObjFile("model/hand/middle_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[13]->setObject(CreateObject(loadObjFile("model/hand/middle_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[14]->setObject(CreateObject(loadObjFile("model/hand/middle_2.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));

	// ring
	_hand->getAllFrameList()[16]->setObject(CreateObject(loadObjFile("model/hand/ring_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[17]->setObject(CreateObject(loadObjFile("model/hand/ring_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[18]->setObject(CreateObject(loadObjFile("model/hand/ring_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[19]->setObject(CreateObject(loadObjFile("model/hand/ring_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));

	// pinky
	_hand->getAllFrameList()[21]->setObject(CreateObject(loadObjFile("model/hand/pinky_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[22]->setObject(CreateObject(loadObjFile("model/hand/pinky_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[23]->setObject(CreateObject(loadObjFile("model/hand/pinky_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
	_hand->getAllFrameList()[24]->setObject(CreateObject(loadObjFile("model/hand/pinky_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));

	_hand->setObjectPose();
	_hand->setFingerRelation();

	//CreateObject(loadObjFile("model/Cup.obj"));
	CreateObject(new btSphereShape(30));
	//CreateObject(new btBoxShape(btVector3(30, 30, 30)));
}

void ContactSimulator::CheckForCollisionEvents(){
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
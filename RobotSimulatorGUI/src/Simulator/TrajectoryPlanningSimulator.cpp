#include "stdafx.h"
#include "TrajectoryPlanningSimulator.h"
#include <fstream>
using namespace std;
using namespace tp;
using namespace Eigen;

TrajectoryPlanningSimulator::TrajectoryPlanningSimulator(){}

TrajectoryPlanningSimulator::~TrajectoryPlanningSimulator(){}

void TrajectoryPlanningSimulator::Keyboard(unsigned char key, int x, int y){
	double score = 0;
	vector<vector<double>> wrenchSpace;
	ArmRRTManager::Tree Ta, Tb;
	ArmRRTManager::Node qinit, qgoal;
	switch (key){
	case 'g':
		while (score <= 0){
			_graspPoseGenerator->generate();
			_graspGenerator->grasp();
			delete _wrenchGenerator;
			delete _qualityMeasure;
			_wrenchGenerator = new WrenchGenerator(_target, &_contactMgr, _frictionCone);
			_qualityMeasure = new GraspWrenchSpaceQualityMeasure();
			_wrenchGenerator->Union(wrenchSpace);
			if (_qualityMeasure->compute(wrenchSpace)){
				score = _qualityMeasure->getEps();
			}
		}
		cout << score << endl;
		cout << "Arm Joint Value:" << endl;
		for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
			cout << (*_arm)[i].getCmd()*180/M_PI << " ";
		}
		cout << endl;
		break;
	case 'd':
		_rrt->setDebugDrawFlag(!_rrt->getDebugDrawFlag());
		break;
	case 't':
		_path.clear();
		qinit.getData() = Eigen::VectorXd::Zero(_arm->DOFsize());
		qgoal.getData() = Eigen::VectorXd::Zero(_arm->DOFsize());
		for (int i = 0, n = _arm->DOFsize(); i < n ; ++i){
			qgoal.getData()[i] = (*_arm)[i].getCmd();
		}
		_hand->reset();
		(*_hand)[0].setCmd(M_PI / 2);
		_hand->setObjectPose();
		_rrt->setEps(0.02);
		_rrt->setIterMax(2000);
		if (_rrt->RRTConnectPlanner(&Ta, &Tb, qinit, qgoal, _path)){
			cout << _path.size() << endl;
			_run = true;
		}
		else{
			cout << "false" << endl;
		}
	}

}

void TrajectoryPlanningSimulator::Special(int key, int x, int y){
	if (key == GLUT_KEY_PAGE_UP){
		_smoother->planning(_path, _path);
		_run = true;
	}
}

void TrajectoryPlanningSimulator::Idle(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	UpdateCamera();

	_rrt->draw();

	if (_run){
		static int j = 0;
		_arm->setTh(_path[j]);
		_hand->connectToArm(_arm);
		j++;
		if (j == _path.size()){
			_run = false;
			_hand->setMotionFlag(true);
			j = 0;
		}
	}

	if (_hand->getMotionFlag()){
		_hand->grasp();
		CheckForCollisionEvents();
	}

	RenderScene();

	glutSwapBuffers();
}

void TrajectoryPlanningSimulator::InitRobotEnv(){
	// init robot
	Simulator::InitRobotEnv();
	(*_hand)[0].setCmd(M_PI / 2);
	_hand->setObjectPose();

	// init env object
	Matrix4d T = Matrix4d::Identity();
	T(0, 3) = 830;
	T(2, 3) = -130;
	_table = CreateObject(new btBoxShape(btVector3(500, 350, 150)), btVector3(0.0, 0.6, 1.0), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	T(0, 3) = 430;
	T(1, 3) = 100;
	T(2, 3) = 200;
	//T(2, 3) = 72;
	CreateObject(loadObjFile("model/Cup.obj"), btVector3(1.0f, 1.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	T(0, 3) = 530;
	T(1, 3) = 180;
	T(2, 3) = 89;
	_target = CreateObject(loadObjFile("model/bottle.obj"), btVector3(1.0f, 1.0f, 0.0f), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB);
	// init FrictionCone
	_frictionCone = new FrictionCone();
	// init GraspPoseGenerator
	_graspPoseGenerator = new GraspPoseGenerator(m_pWorld, _arm, _hand, _target);
	// init GraspGeneraor
	_graspGenerator = new GraspGenerator(m_pWorld, _hand, _target, &_contactMgr);
	// init rrt manager
	_rrt = new ArmRRTManager(m_pWorld, _arm, _hand);
	// init rrt smoother
	_smoother = new RRTSmoother(m_pWorld, _arm, _hand);
}

bool TrajectoryPlanningSimulator::checkCollision(){
	//Perform collision detection // 這行一定要有!
	m_pWorld->performDiscreteCollisionDetection();
	int numManifolds = m_pWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = m_pWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)contactManifold->getBody0();
		btCollisionObject* obB = (btCollisionObject*)contactManifold->getBody1();
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		if (contactManifold->getNumContacts()){
			return true;
		}
	}
	return false;
}
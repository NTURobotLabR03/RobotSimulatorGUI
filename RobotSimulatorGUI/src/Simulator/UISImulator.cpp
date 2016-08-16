#include "stdafx.h"
#include "UISimulator.h"
#include "ThreadControl.h"
using namespace Eigen;

UISimulator::UISimulator(){

}

UISimulator::~UISimulator(){

}

void UISimulator::Keyboard(unsigned char key, int x, int y){

}

void UISimulator::Special(int key, int x, int y){

}

void UISimulator::Idle(){
	lock_guard<mutex> mLock(gMutex);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	UpdateCamera();

	RenderScene();

	aidFrameDraw();

	allDebugDraw();

	glutSwapBuffers();
}

void UISimulator::allDebugDraw(){
	_rrt->draw();
	_contactMgr.draw();

	if (_pathFlag){
		glColor3f(1.0, 0.0, 0.0);
		for (auto& it : _pathFK){
			glPushMatrix();
			glTranslated(it[0], it[1], it[2]);
			glutSolidSphere(3, 4, 4);
			glPopMatrix();
		}
	}
}

void UISimulator::aidFrameDraw(){
	if (_aidFlag){
		glPushMatrix();
		glMultMatrixd(_aidFrame.data());
		Draw_Cute_Axis(30);
		glPopMatrix();
	}
}

const std::vector<Eigen::VectorXd>& UISimulator::getPath() const{
	return _path;
}

void UISimulator::solveIK(){
	vector<double> th;
	_arm->dlsIK(_aidFrame, th);
	for (int i = 0, n = th.size(); i < n; ++i){
		(*_arm)[i].setCmd(th[i]);
	}
	_arm->setObjectPose();
	_hand->connectToArm(_arm);
}

void UISimulator::setAidFlag(bool flag){
	_aidFlag = flag;
}

void UISimulator::setRRTDrawFlag(bool flag){
	_rrt->setDebugDrawFlag(flag);
}

void UISimulator::setContactMgrDrawFlag(bool flag){
	_contactMgr.setDebugDrawFlag(flag);
}

void UISimulator::setDrawAabbFlag(){
	m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawAabb);
}

double UISimulator::graspPlanning(){
	if (_target){
		//lock_guard<mutex> mLock(gMutex);
		int iter = 0, maxIter = 100;
		double score = 0;
		vector<vector<double>> wrenchSpace;

		while (score <= 0 && iter < maxIter){
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
			iter++;
		}

		if (score > 0){
			//cout << score << endl;
			//cout << "Arm Joint Value:" << endl;
			//for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
			//	cout << (*_arm)[i].getCmd() * 180 / M_PI << " ";
			//}
			//cout << endl;
		}
		else{
			cout << "false" << endl;
		}
		return score;
	}
	return -1.0;
}

void UISimulator::pathPlanning(const HandArmState& Qinit, const HandArmState& Qgoal){

	{
		lock_guard<mutex> mLock(gMutex);
		// RRT
		if (Qinit.getIsGrasp()){
			for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
				(*_arm)[i].setCmd(Qinit.getArmTh()[i]);
			}
			_arm->setObjectPose();
			if (_target){
				_arm->setStickObject(_target);
			}
		}

		ArmRRTManager::Tree Ta, Tb;
		ArmRRTManager::Node qinit, qgoal;

		_path.clear();
		qinit.getData() = Eigen::VectorXd::Zero(_arm->DOFsize());
		qgoal.getData() = Eigen::VectorXd::Zero(_arm->DOFsize());
		for (int i = 0, n = Qinit.getArmTh().size(); i < n; ++i){
			qinit.getData()[i] = Qinit.getArmTh()[i];
			qgoal.getData()[i] = Qgoal.getArmTh()[i];
		}
		_hand->reset();
		(*_hand)[0].setCmd(M_PI / 2);
		_hand->setObjectPose();
		_rrt->setEps(0.02);
		_rrt->setIterMax(5000);
		if (_rrt->RRTConnectPlanner(&Ta, &Tb, qinit, qgoal, _path)){
			// smoother
			_smoother->planning(_path, _path);
			_pathFK.clear();
			for (auto& it : _path){
				for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
					(*_arm)[i].setCmd(it[i]);
				}
				_pathFK.push_back(Vector3d());
				_arm->getFK(_pathFK.back());
			}
			_run = true;
		}
		else{
			cout << "rrt false" << endl;
		}
	}

	if (Qinit.getIsGrasp()){
		for (int i = 0, n = Qinit.getHandTh().size(); i < n; ++i){
			(*_hand)[i].setCmd(Qinit.getHandTh()[i]);
		}
		_hand->setObjectPose();
	}

	while (_run){
		lock_guard<mutex> mLock(gMutex);
		this_thread::sleep_for(chrono::duration<int, milli>(5));
		static int j = 0;
		Matrix4d base2Arm;
		_arm->setTh(_path[j]);
		_hand->connectToArm(_arm);
		j++;
		if (j == _path.size()){
			_arm->clearStickObject();
			_run = false;
			j = 0;
		}
	}
}

void UISimulator::pathSmoothing(){
	{
		lock_guard<mutex> mLock(gMutex);
		_smoother->planning(_path, _path);
		_pathFK.clear();
		for (auto& it : _path){
			for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
				(*_arm)[i].setCmd(it[i]);
			}
			_pathFK.push_back(Vector3d());
			_arm->getFK(_pathFK.back());
		}
		_run = true;
	}
	while (_run){
		lock_guard<mutex> mLock(gMutex);
		this_thread::sleep_for(chrono::duration<int, milli>(5));
		static int j = 0;
		_arm->setTh(_path[j]);
		_hand->connectToArm(_arm);
		j++;
		if (j == _path.size()){
			_run = false;
			j = 0;
		}
	}
}

void UISimulator::setPathFlag(bool flag){
	_pathFlag = flag;
}

void UISimulator::handGrasp(){
	_hand->closeThumbFinger();
	_hand->setMotionFlag(true);
	while (_hand->getMotionFlag()){
		lock_guard<mutex> mLock(gMutex);
		this_thread::sleep_for(chrono::duration<int, milli>(1));
		_hand->grasp();
		CheckForCollisionEvents();
	}
}

void UISimulator::handRelease(){
	_hand->release();
}

Object* UISimulator::getTable(){
	return _table;
}

void UISimulator::deleteObject(Object* obj){
	vector<Object*>::iterator it = find(m_objects.begin(), m_objects.end(), obj);
	m_pWorld->removeCollisionObject(obj->getCollisionObject());
	delete *it;
	m_objects.erase(it);
}

void UISimulator::setTarget(Object* object){
	_target = object;
	if (_target){
		delete _graspPoseGenerator;
		_graspPoseGenerator = new GraspPoseGenerator(m_pWorld, _arm, _hand, _target);
		delete _graspGenerator;
		_graspGenerator = new GraspGenerator(m_pWorld, _hand, _target, &_contactMgr);
	}
	else{
		delete _graspPoseGenerator;
		_graspPoseGenerator = nullptr;
		delete _graspGenerator;
		_graspGenerator = nullptr;
	}
}

Object* UISimulator::getTarget(){
	return _target;
}

void UISimulator::tempDeleteObject(Object* obj){
	m_pWorld->removeCollisionObject(obj->getCollisionObject());
}

void UISimulator::setArmJoint(const std::vector<double>& values){
	for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
		(*_arm)[i].setCmd(values[i]);
	}
	_arm->setObjectPose();
	_hand->connectToArm(_arm);
}

void UISimulator::setArmJoint(const Eigen::VectorXd& values){
	_arm->setTh(values);
	_hand->connectToArm(_arm);
}

void UISimulator::getArmJoint(std::vector<double>& values){
	values.clear();
	for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
		values.push_back((*_arm)[i].getCmd());
	}
}

void UISimulator::getArmJoint(Eigen::VectorXd& values){
	values = Eigen::VectorXd(_arm->DOFsize());
	for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
		values[i] = (*_arm)[i].getCmd();
	}
}

void UISimulator::getHandJoint(std::vector<double>& values){
	values.clear();
	for (int i = 0, n = _hand->DOFsize(); i < n; ++i){
		values.push_back((*_hand)[i].getCmd());
	}
}

void UISimulator::getHandJoint(Eigen::VectorXd& values){
	values = Eigen::VectorXd(_hand->DOFsize());
	for (int i = 0, n = _hand->DOFsize(); i < n; ++i){
		values[i] = (*_hand)[i].getCmd();
	}
}

void UISimulator::setHandJoint(const std::vector<double>& values){
	for (int i = 0, n = _hand->DOFsize(); i < n; ++i){
		(*_hand)[i].setCmd(values[i]);
	}
	_hand->setObjectPose();
}

void UISimulator::setHandJoint(const Eigen::VectorXd& values){
	for (int i = 0, n = _hand->DOFsize(); i < n; ++i){
		(*_hand)[i].setCmd(values[i]);
	}
	_hand->setObjectPose();
}

void UISimulator::getArmFK(Eigen::Matrix4d& T){
	_arm->getFK(T);
}

Eigen::Matrix4d& UISimulator::getAidFrame(){
	return _aidFrame;
}

void UISimulator::restoreObject(Object* obj){
	m_pWorld->addCollisionObject(obj->getCollisionObject(), COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV);
}

void UISimulator::runPath(PathPlayBack* path){
	for (int i = 0, n = _arm->DOFsize(); i < n; ++i){
		(*_arm)[i].setCmd(path->getQinit().getArmTh()[i]);
	}
	for (int i = 0, n = _hand->DOFsize(); i < n; ++i){
		(*_hand)[i].setCmd(path->getQinit().getHandTh()[i]);
	}
	if (path->getObj()){
		path->getObj()->setCOM6D(path->getObjT0());
		_arm->setStickObject(path->getObj());
	}
	bool run = true;
	int j = 0;
	while (run){
		lock_guard<mutex> mLock(gMutex);
		this_thread::sleep_for(chrono::duration<int, milli>(5));
		_arm->setTh(path->getPath()[j]);
		_hand->connectToArm(_arm);
		j++;
		if (j == path->getPath().size()){
			run = false;
			_arm->clearStickObject();
		}
	}

	if (path->getQgoal().getIsGrasp()){
		if (!path->getQinit().getIsGrasp()){
			handGrasp();
			this_thread::sleep_for(chrono::duration<int, milli>(1000));
		}
	}
	else{
		handRelease();
	}
}

void UISimulator::InitRobotEnv(){
	// init robot
	Simulator::InitRobotEnv();
	(*_hand)[0].setCmd(M_PI / 2);
	_hand->setObjectPose();

	// init env object
	Matrix4d T = Matrix4d::Identity();
	T(0, 3) = 830;
	T(2, 3) = -130;
	_table = CreateObject(new btBoxShape(btVector3(500, 350, 150)), btVector3(0.0, 0.6, 1.0), T, COLLISIONGROUP_ENV, COLLISIONGROUP_ARM | COLLISIONGROUP_HANDBASE | COLLISIONGROUP_FINGER | COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV);
	// init FrictionCone
	_frictionCone = new FrictionCone();
	// init rrt manager
	_rrt = new ArmRRTManager(m_pWorld, _arm, _hand);
	// init rrt smoother
	_smoother = new RRTSmoother(m_pWorld, _arm, _hand);
}
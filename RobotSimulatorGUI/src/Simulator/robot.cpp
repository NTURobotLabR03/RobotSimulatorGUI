#include "stdafx.h"
#include "robot.h"
using namespace rbt;
using namespace Eigen;
//=======================================================Robot=================================================
rbt::Robot::Robot(){
	_allFrameList.push_back(new BasicFrame()); 
	_baseFrameList.push_back(_allFrameList.back());
}

rbt::Robot::Robot(const Eigen::Matrix4d& T){
	_allFrameList.push_back(new BasicFrame(T)); 
	_baseFrameList.push_back(_allFrameList.back());
}

rbt::Robot::~Robot(){
	for (int i = 0, n = _allFrameList.size(); i < n; ++i){
		delete _allFrameList[i];
	}
	_allFrameList.clear();
	_baseFrameList.clear();
	_jointFrameList.clear();
	_passiveFrameList.clear();
	for (int i = 0, n = _endEffectorList.size(); i < n; ++i){
		delete _endEffectorList[i];
	}
	_endEffectorList.clear();
}

void rbt::Robot::addBasicFrame(const Eigen::Matrix4d& T, int parentId){
	_allFrameList.push_back(new BasicFrame(T));
	if (parentId == -1)
		_baseFrameList.push_back(_allFrameList.back());
	else{
		_allFrameList[parentId]->addChild(_allFrameList.back());
	}
}

void rbt::Robot::addRevoluteFrame(double a, double alpha, double d, double theta, double min, double max, int parentId){
	_jointFrameList.push_back(new DHFrame(a, alpha, d, theta, min, max));
	_jointFrameList.back()->setId(_jointFrameList.size() - 1);
	_allFrameList.push_back(_jointFrameList.back());
	if (parentId == -1)
		_baseFrameList.push_back(_allFrameList.back());
	else{
		_allFrameList[parentId]->addChild(_allFrameList.back());
	}
}

void rbt::Robot::addPassiveFrame(double a, double alpha, double d, double theta, double ratio, int activeParentId, int parentId){
	_passiveFrameList.push_back(new PDHFrame(a, alpha, d, theta, ratio, ((DHFrame*)_allFrameList[activeParentId])));
	_allFrameList.push_back(_passiveFrameList.back());
	((DHFrame*)_allFrameList[activeParentId])->addPassiveChild(_passiveFrameList.back());
	if (parentId == -1)
		_baseFrameList.push_back(_allFrameList.back());
	else
		_allFrameList[parentId]->addChild(_allFrameList.back());
}

void rbt::Robot::searchEE(){
	_endEffectorList.clear();
	std::vector<Frame*> path;
	for (int i = 0, n = _baseFrameList.size(); i < n; ++i){
		_baseFrameList[i]->searchEE(path, _endEffectorList);
	}
}

void rbt::Robot::drawRobot() const{
	Draw_Cute_Axis(15);
	for (int i = 0, n = _baseFrameList.size(); i < n; ++i){
		glPushMatrix();
			_baseFrameList[i]->draw();
		glPopMatrix();
	}
}

unsigned rbt::Robot::DOFsize() const{
	return _jointFrameList.size();
}

unsigned rbt::Robot::EEsize() const{ 
	return _endEffectorList.size(); 
}

const DHFrame& rbt::Robot::operator[] (unsigned i) const{
	return *_jointFrameList[i];
}

DHFrame& rbt::Robot::operator[] (unsigned i){ 
	return *_jointFrameList[i];
}

void rbt::Robot::getFK(std::vector<Eigen::Vector3d>& endEffectorPos) const{
	endEffectorPos.clear();
	for (int i = 0, n = _baseFrameList.size(); i < n; ++i){
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		_baseFrameList[i]->getFK(endEffectorPos, T);
	}
}

void rbt::Robot::getFK(Eigen::Matrix4dList& endEffectorMat) const{
	endEffectorMat.clear();
	for (int i = 0, n = _baseFrameList.size(); i < n; ++i){
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		_baseFrameList[i]->getFK(endEffectorMat, T);
	}
}

void rbt::Robot::getJacobian(Eigen::MatrixXd& J) const{
	J = Eigen::MatrixXd::Zero(_endEffectorList.size() * 3, _jointFrameList.size());
	std::vector<Eigen::Vector3d> Pe;
	getFK(Pe);
	for (int i = 0, n = _endEffectorList.size(); i < n; ++i){
		Eigen::Matrix4d TransMat = Eigen::Matrix4d::Identity();
		for (int j = 0, m = _endEffectorList[i]->getPath().size(); j < m; ++j){
			if (_endEffectorList[i]->getPath()[j]->isDrive()){
				Eigen::Vector3d& element = TransMat.block<3, 1>(0, 2).cross(Pe[i] - TransMat.block<3, 1>(0, 3));
				J.block<3, 1>(3 * i, ((DHFrame*)_endEffectorList[i]->getPath()[j])->getId()) += element;
			}
			else if (_endEffectorList[i]->getPath()[j]->isPassive()){
				Eigen::Vector3d& element = TransMat.block<3, 1>(0, 2).cross(Pe[i] - TransMat.block<3, 1>(0, 3));
				J.block<3, 1>(3 * i, ((PDHFrame*)_endEffectorList[i]->getPath()[j])->getDriveId()) += ((PDHFrame*)_endEffectorList[i]->getPath()[j])->getRatio()*element;
			}
			TransMat *= _endEffectorList[i]->getPath()[j]->getTransMat();
		}
	}
}

void rbt::Robot::getJacobianWO(Eigen::MatrixXd& J) const{
	J = Eigen::MatrixXd::Zero(_endEffectorList.size() * 6, _jointFrameList.size());
	std::vector<Eigen::Vector3d> Pe;
	getFK(Pe);
	for (int i = 0, n = _endEffectorList.size(); i < n; ++i){
		Eigen::Matrix4d TransMat = Eigen::Matrix4d::Identity();
		for (int j = 0, m = _endEffectorList[i]->getPath().size(); j < m; ++j){
			if (_endEffectorList[i]->getPath()[j]->isDrive()){
				Eigen::Vector3d element = TransMat.block<3, 1>(0, 2).cross(Pe[i] - TransMat.block<3, 1>(0, 3));
				J.block<3, 1>(6 * i, ((DHFrame*)_endEffectorList[i]->getPath()[j])->getId()) += element;
				J.block<3, 1>(6 * i + 3, ((DHFrame*)_endEffectorList[i]->getPath()[j])->getId()) += TransMat.block<3, 1>(0, 2);
			}
			else if (_endEffectorList[i]->getPath()[j]->isPassive()){
				Eigen::Vector3d element = TransMat.block<3, 1>(0, 2).cross(Pe[i] - TransMat.block<3, 1>(0, 3));
				J.block<3, 1>(6 * i, ((PDHFrame*)_endEffectorList[i]->getPath()[j])->getDriveId()) += ((PDHFrame*)_endEffectorList[i]->getPath()[j])->getRatio()*element;
				J.block<3, 1>(6 * i + 3, ((PDHFrame*)_endEffectorList[i]->getPath()[j])->getDriveId()) += ((PDHFrame*)_endEffectorList[i]->getPath()[j])->getRatio()*TransMat.block<3, 1>(0, 2);
			}
			TransMat *= _endEffectorList[i]->getPath()[j]->getTransMat();
		}
	}
}

void rbt::Robot::reset(){
	for (int i = 0, n = _jointFrameList.size(); i < n; ++i)
		_jointFrameList[i]->setCmd(0);
}

void rbt::Robot::dlsIK(const std::vector<Eigen::Vector3d>& targetPos, std::vector<double>& jointValue){
	jointValue.clear();
	// declare
	Eigen::MatrixXd J, JJt;
	double lambda = 100; // 自己調的
	static const unsigned maxIter = 1500;
	static const double Dmax = 10, eps = 0.1;
	std::vector<double> initialCmd;
	std::vector<Eigen::Vector3d> endEffectorPos, err;
	Eigen::VectorXd errVec(targetPos.size() * 3);
	Eigen::VectorXd delTh(DOFsize());
	double errNorm = 0;
	// first compute 
	for (int i = 0, n = DOFsize(); i < n; ++i){
		initialCmd.push_back(_jointFrameList[i]->getCmd());
	}
	unsigned iter = 0;
	getFK(endEffectorPos);
	for (int i = 0, n = targetPos.size(); i < n; ++i){
		err.push_back(targetPos[i] - endEffectorPos[i]);
		errNorm += err.back().norm();
	}
	while (iter < maxIter && errNorm > eps){
		errNorm = 0;
		// Clamp Mag
		for (int i = 0, n = err.size(); i < n; ++i){
			if (err[i].norm() > Dmax){
				err[i] = err[i] * Dmax / err[i].norm();
			}
			for (int j = 0; j < 3; ++j){
				errVec[3 * i + j] = err[i][j];
			}
		}
		err.clear();
		getJacobian(J);
		J = J.topRows(targetPos.size() * 3).eval();
		JJt = J*J.transpose();
		delTh = J.transpose()*(JJt + pow(lambda, 2)*Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols())).inverse()*errVec;
		for (int i = 0, n = DOFsize(); i < n; ++i){
			_jointFrameList[i]->updateCmd(delTh[i]);
		}
		getFK(endEffectorPos);
		for (int i = 0, n = targetPos.size(); i < n; ++i){
			err.push_back(targetPos[i] - endEffectorPos[i]);
			errNorm += err.back().norm();
		}
		iter++;
	}
	for (int i = 0, n = DOFsize(); i < n; ++i){
		jointValue.push_back(_jointFrameList[i]->getCmd());
		_jointFrameList[i]->setCmd(initialCmd[i]);
	}
}

void rbt::Robot::dlsIK(const Eigen::Matrix4dList& targetPosMat, std::vector<double>& jointValue){
	jointValue.clear();
	// declare
	Eigen::MatrixXd J, JJt;
	double lambda = 5*targetPosMat.size(); // 自己調的
	static const unsigned maxIter = 10000;
	static double Dmax = 200, eps = 0.001;
	std::vector<double> initialCmd;
	for (int i = 0, n = DOFsize(); i < n; ++i){
		initialCmd.push_back(_jointFrameList[i]->getCmd());
	}
	Eigen::Matrix4dList endEffectorMat;
	std::vector<Eigen::VectorXd> err;
	Eigen::VectorXd errVec(targetPosMat.size() * 6);
	Eigen::VectorXd delTh(DOFsize());
	double errNorm = 0;
	unsigned iter = 0;
	getFK(endEffectorMat);
	for (int i = 0, n = targetPosMat.size(); i < n; ++i){
		Eigen::VectorXd endEffectorVec, targetVec;
		transMat2Vec(targetPosMat[i], targetVec);
		transMat2Vec(endEffectorMat[i], endEffectorVec);
		err.push_back(targetVec-endEffectorVec);
		errNorm += err.back().norm();
	}
	while (iter < maxIter && errNorm > eps){
		errNorm = 0;
		// Clamp Mag
		for (int i = 0, n = err.size(); i < n; ++i){
			if (err[i].norm() > Dmax){
				err[i] = err[i] * Dmax / err[i].norm();
			}
			for (int j = 0; j < 6; ++j){
				errVec[6 * i + j] = err[i][j];
			}
		}
		err.clear();
		getJacobianWO(J);
		J = J.topRows(targetPosMat.size() * 6).eval();
		JJt = J*J.transpose();
		delTh = J.transpose()*(JJt + pow(lambda, 2)*Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols())).inverse()*errVec;
		for (int i = 0, n = J.cols(); i < n; ++i){
			_jointFrameList[i]->updateCmd(delTh[i]);
		}
		getFK(endEffectorMat);
		for (int i = 0, n = targetPosMat.size(); i < n; ++i){
			Eigen::VectorXd endEffectorVec, targetVec;
			transMat2Vec(targetPosMat[i], targetVec);
			transMat2Vec(endEffectorMat[i], endEffectorVec);
			err.push_back(targetVec - endEffectorVec);
			errNorm += err.back().norm();
		}
		iter++;
	}
	for (int i = 0, n = DOFsize(); i < n; ++i){
		jointValue.push_back(_jointFrameList[i]->getCmd());
		_jointFrameList[i]->setCmd(initialCmd[i]);
	}
}

// 20160421
std::vector<Frame*>& rbt::Robot::getAllFrameList(){
	return _allFrameList;
}

void rbt::Robot::setObjectPose(){
	Matrix4d T = Matrix4d::Identity();
	for (int i = 0, n = _baseFrameList.size(); i < n; ++i){
		_baseFrameList[i]->setObjectPose(T);
	}
}


//=============================================================================================================
//=====================================================Other function==========================================
void rbt::transMat2Vec(const Eigen::Matrix4d& transMat, Eigen::VectorXd& P){
	P = Eigen::VectorXd::Zero(6);
	for (int i = 0; i < 3; ++i){
		P(i) = transMat(i, 3);
	}
	double th;
	th = transMat(0, 0) + transMat(1, 1) + transMat(2, 2) - 1;
	th = acos(th / 2);
	Eigen::Vector3d k;
	if (th > 0.001){
		k << (transMat(2, 1) - transMat(1, 2)),
			(transMat(0, 2) - transMat(2, 0)),
			(transMat(1, 0) - transMat(0, 1));
		k = k / (2 * sin(th));
		k = th*k;
	}
	else{
		k = Eigen::Vector3d::Zero();
	}
	for (int i = 3; i < 6; ++i){
		P(i) = k(i - 3);
	}
}
//=============================================================================================================
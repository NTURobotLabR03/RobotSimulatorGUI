#include "stdafx.h"
#include "PathPlayBack.h"
#include <iostream>
using namespace std;
PathPlayBack::PathPlayBack(){

}

PathPlayBack::PathPlayBack(const HandArmState& Qinit, const HandArmState& Qgoal, const std::vector<Eigen::VectorXd> path, Object* obj, Eigen::Matrix4d T0){
	_Qinit = Qinit;
	_Qgoal = Qgoal;
	_path = path;
	_obj = obj;
	_T0 = T0;
}

PathPlayBack::~PathPlayBack(){

}

const std::vector<Eigen::VectorXd>& PathPlayBack::getPath() const{
	return _path;
}

const HandArmState& PathPlayBack::getQinit() const{
	return _Qinit;
}

const HandArmState& PathPlayBack::getQgoal() const{
	return _Qgoal;
}

const Eigen::Matrix4d& PathPlayBack::getObjT0() const{
	return _T0;
}

Object* PathPlayBack::getObj(){
	return _obj;
}
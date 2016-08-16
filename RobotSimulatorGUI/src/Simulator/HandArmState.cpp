#include "stdafx.h"
#include "HandArmState.h"
HandArmState::HandArmState(){

}

HandArmState::~HandArmState(){

}

std::vector<double>& HandArmState::getArmTh(){
	return _armTh;
}

std::vector<double>& HandArmState::getHandTh(){
	return _handTh;
}

const std::vector<double>& HandArmState::getArmTh() const{
	return _armTh;
}

const std::vector<double>& HandArmState::getHandTh() const{
	return _handTh;
}

void HandArmState::setIsGrasp(bool flag){
	_isGrasp = flag;
}

bool HandArmState::getIsGrasp() const{
	return _isGrasp;
}
#include "stdafx.h"
#include "ContactMgr.h"
#include <iostream>
#include <algorithm>
using namespace std;
using namespace Eigen;

ContactMgr::ContactMgr(){}
ContactMgr::~ContactMgr(){}

void ContactMgr::push_back(ContactInfo* info){
	_contactInfos.push_back(info);
}

void ContactMgr::clear(){
	// 會做delete所以不用自己delete
	for (auto& it : _contactInfos){
		delete it;
	}
	_contactInfos.clear();
}

ContactInfo* ContactMgr::operator[](int i){
	return _contactInfos[i];
}

size_t ContactMgr::size(){
	return _contactInfos.size();
}

bool ContactMgr::getDebugDrawFlag(){
	return _debugDrawFlag;
}

void ContactMgr::setDebugDrawFlag(bool flag){
	_debugDrawFlag = flag;
}

void ContactMgr::draw(){
	if (_debugDrawFlag){
		for (auto& it : _contactInfos){
			it->draw();
		}
	}
}

void ContactMgr::analysis(const Object* Object){
	vector<ContactInfo*> contactInfos;
	Vector3d com = Object->getCOM3D();
	Vector3d normal;
	Vector3d fingerPowerOrienX, fingerPowerOrienY;
	for (auto& it : _contactInfos){
		if (it->getCollisionPair().second == Object->getCollisionObject() || it->getCollisionPair().first == Object->getCollisionObject()){
			if (it->getDistance() >= _eps){
				normal << it->getNormal()[0], it->getNormal()[1], it->getNormal()[2];
				btScalar temp[16];
				it->getCollisionPair().first->getWorldTransform().getOpenGLMatrix(temp);
				fingerPowerOrienX << temp[0], temp[1], temp[2];
				fingerPowerOrienY << temp[4], temp[5], temp[6];

				double xdot = fingerPowerOrienX.dot(normal), ydot = fingerPowerOrienY.dot(normal);
				if (abs(xdot) > abs(ydot)){
					if (xdot > 0){
						it->setNormalInv();
					}
				}
				else if (abs(ydot) > abs(xdot)){
					if (ydot > 0){
						it->setNormalInv();
					}
				}
				//if (fingerPowerOrienY.dot(normal) > 0){
				//	it->setNormalInv();
				//}
				contactInfos.push_back(new ContactInfo(*it));
			}
		}
	}

	this->clear();
	_contactInfos = contactInfos;
}
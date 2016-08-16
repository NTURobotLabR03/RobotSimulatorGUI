#include "stdafx.h"
#include "ContactInfo.h"
#include "FreeGlutHeader.h"

ContactInfo::ContactInfo(){}
ContactInfo::~ContactInfo(){}

ContactInfo::ContactInfo(const CollisionPair& pair, const btVector3& point, const btVector3& normal, double distance){
	_pair = pair;
	_point = point;
	_normal = normal;
	_distance = distance;
}

const btVector3& ContactInfo::getPoint() const{
	return _point;
}

const btVector3& ContactInfo::getNormal() const{
	return _normal;
}

const CollisionPair& ContactInfo::getCollisionPair(){
	return _pair;
}

void ContactInfo::draw(){
	glPushMatrix();
	glTranslatef(_point[0], _point[1], _point[2]);
	glColor3f(1.0, 1.0, 0.0);
	glutSolidSphere(1, 10, 10);
	glColor3f(1.0, 0.0, 0.0);
	//Arrow(-_normal[0] * 10, -_normal[1] * 10, -_normal[2] * 10, 0.0, 0.0, 0.0, 3);
	Arrow(_normal[0] * 10, _normal[1] * 10, _normal[2] * 10, 0.0, 0.0, 0.0, 3);
	glPopMatrix();
}

double ContactInfo::getDistance(){
	return _distance;
}

void ContactInfo::setNormalInv(){
	_normal[0] = -_normal[0];
	_normal[1] = -_normal[1];
	_normal[2] = -_normal[2];
}
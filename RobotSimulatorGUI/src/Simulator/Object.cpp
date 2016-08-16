#include "stdafx.h"
#include "Object.h"
Object::Object(){}

Object::Object(btCollisionShape* pShape, const btVector3& color, const Eigen::Matrix4d& transMat){
	// store the shape for later usage
	m_pShape = pShape;

	// store the color
	m_color = color;

	m_pObject = new btCollisionObject();
	m_pObject->setCollisionShape(m_pShape);
	
	btTransform transform;
	btScalar temp[16];
	for (int i = 0; i < 16; ++i){
		temp[i] = transMat.data()[i];
	}

	transform.setFromOpenGLMatrix(temp);
	m_pObject->setWorldTransform(transform);
}

Object::~Object(){
	delete m_pObject;
	delete m_pShape;
}

btCollisionObject* Object::getCollisionObject() const{
	return m_pObject;
}

btCollisionShape* Object::getShape(){
	return m_pShape;
}

const btVector3& Object::getColor(){
	return m_color;
}

void Object::setColor(const btVector3& color){
	m_color = color;
}

Eigen::Vector3d Object::getCOM3D() const{
	Eigen::Vector3d pos;
	pos << m_pObject->getWorldTransform().getOrigin()[0], m_pObject->getWorldTransform().getOrigin()[1], m_pObject->getWorldTransform().getOrigin()[2];
	return pos;
}

void Object::setCOM3D(const Eigen::Vector3d& pos){
	btVector3 temp;
	temp[0] = pos[0];
	temp[1] = pos[1];
	temp[2] = pos[2];
	m_pObject->getWorldTransform().setOrigin(temp);
}

Eigen::Matrix4d Object::getCOM6D(){
	Eigen::Matrix4d transMat;
	btScalar temp[16];
	m_pObject->getWorldTransform().getOpenGLMatrix(temp);
	for (int i = 0; i < 16; ++i){
		transMat.data()[i] = temp[i];
	}
	return transMat;
}

void Object::setCOM6D(const Eigen::Matrix4d& posMat){
	btTransform transform;
	btScalar temp[16];
	for (int i = 0; i < 16; ++i){
		temp[i] = posMat.data()[i];
	}
	transform.setFromOpenGLMatrix(temp);
	m_pObject->setWorldTransform(transform);
}

void Object::setContact(bool flag){
	m_contact = flag;
}

bool Object::getContact(){
	return m_contact;
}

void Object::GetTransform(btScalar* transform){
	m_pObject->getWorldTransform().getOpenGLMatrix(transform);
}

void Object::getaabb(Eigen::Vector3d& aabbMin, Eigen::Vector3d& aabbMax){
	btVector3 AabbMin, AabbMax;
	btTransform T;
	T = m_pObject->getWorldTransform();
	m_pShape->getAabb(T, AabbMin, AabbMax);
	aabbMin << AabbMin[0], AabbMin[1], AabbMin[2];
	aabbMax << AabbMax[0], AabbMax[1], AabbMax[2];
}

void Object::setTempDeleteFlag(bool flag){
	_tempDeleteFlag = flag;
}

bool Object::getTempDeleteFlag(){
	return _tempDeleteFlag;
}
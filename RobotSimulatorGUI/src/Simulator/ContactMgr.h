/*
//���O�W�١GContactMgr
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�N�Ҧ��I�����ͪ�ContactInfo�s�_��
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "ContactInfo.h"
#include "Object.h"
#include <vector>
#include <map>

class ContactMgr{
public:
	ContactMgr();
	~ContactMgr();
	// push�@��ContactInfo
	void push_back(ContactInfo* info);
	// clear��
	void clear();
	// �^�Ǧ��h��ContactInfo
	size_t size();
	// operatorp[] �^�ǲ�i��ContactInfo����
	ContactInfo* operator[](int i);
	// �]�w�n���n�e�bopenGL�W
	bool getDebugDrawFlag();
	void setDebugDrawFlag(bool flag);
	// all draw
	void draw();
	// ��Y��object ���R
	void analysis(const Object* Object);

protected:
	std::vector<ContactInfo*> _contactInfos;
	bool _debugDrawFlag = false;
	double _eps = -1.0;
};
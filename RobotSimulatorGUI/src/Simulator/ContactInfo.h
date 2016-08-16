/*
//���O�W�١GContactInfo
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�N�I������T�s�U�Ӫ���(�I���I�B�I����V(��Ӫk�V�q�����i��)�B����ӪF�輲��)
//�ϥΨ禡�w�GEigen, Bullet
*/
#pragma once
#include "BulletHeader.h"
#include "EigenHeader.h"
#include <map>

typedef std::pair<btCollisionObject*, btCollisionObject*> CollisionPair;

class ContactInfo{
public:
	ContactInfo();
	ContactInfo(const CollisionPair& pair, const btVector3& point, const btVector3& normal, double distance);
	~ContactInfo();
	// �^�ǸI���I
	const btVector3& getPoint() const;
	// �^�ǸI����V
	const btVector3& getNormal() const;
	// �p�G��V���F�i��ncall�o��function
	void setNormalInv(); // �����D�|���|�Ψ�
	// �^�Ǩ�ӸI������
	const CollisionPair& getCollisionPair();
	// �I���Z��(�������I���O���mesh�L��) �t���N��w�gmesh���z�t�~�@�Ӫ���
	double getDistance();
	// �e�X�I���I���V
	void draw();

protected:
	CollisionPair _pair;
	btVector3 _point;
	btVector3 _normal;
	double _distance;
};


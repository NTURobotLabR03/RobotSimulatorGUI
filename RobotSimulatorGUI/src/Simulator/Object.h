/*
//���O�W�١GObject
//�@�̡GChris Dickinson, Liu Yi-Ren
//����G2016/08/15
//�ت��G�NBullet���I���Ϊ�(btCollisionShape), �I������(btCollisionObject), GL�W����C��, ���S���I���L�����_�Ӫ����c
//      ��K�Ω�s�WCAD�ϡB²��X��
//�ϥΨ禡�w�GBullet, Eigen
*/
#pragma once
#include "BulletHeader.h"
#include "EigenHeader.h"

class Object{
public:
	// �غc
	Object();
	Object(btCollisionShape* pShape, const btVector3& color, const Eigen::Matrix4d& transMat);
	// �Ѻc
	~Object();

	// accessors
	// �^��bullet �Ϊ���
	btCollisionShape* getShape();
	// �^��bullet ������
	btCollisionObject* getCollisionObject() const;
	// �^���C�� �eopenGL��
	const btVector3& getColor();
	// �]�w�C��
	void setColor(const btVector3& color);

	// this transform for gl draw
	void GetTransform(btScalar* transform);
	// �o�쪫���ߦ�m
	Eigen::Vector3d getCOM3D() const;
	void setCOM3D(const Eigen::Vector3d& pos);
	Eigen::Matrix4d getCOM6D();
	// �o�쪫���߫��A(position+orientation)
	void setCOM6D(const Eigen::Matrix4d& posMat);
	// �]�w�O�_�I�� �ΦbCheckForCollisionEvents()�̭���reset
	void setContact(bool flag);
	// �^�ǬO�_�I��
	bool getContact();
	// �o��AABB
	void getaabb(Eigen::Vector3d& aabbMin, Eigen::Vector3d& aabbMax);
	// �]�w�O�_�ȮɧR��
	void setTempDeleteFlag(bool flag);
	// �o���O�_�ȮɧR��
	bool getTempDeleteFlag();

protected:
	btCollisionShape* m_pShape;
	btCollisionObject* m_pObject;
	btVector3 m_color;
	bool m_contact;
	bool _tempDeleteFlag = false;
};
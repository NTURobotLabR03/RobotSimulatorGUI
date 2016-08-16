/*
//類別名稱：Object
//作者：Chris Dickinson, Liu Yi-Ren
//日期：2016/08/15
//目的：將Bullet的碰撞形狀(btCollisionShape), 碰撞物體(btCollisionObject), GL上顯示顏色, 有沒有碰撞過紀錄起來的結構
//      方便用於新增CAD圖、簡單幾何
//使用函式庫：Bullet, Eigen
*/
#pragma once
#include "BulletHeader.h"
#include "EigenHeader.h"

class Object{
public:
	// 建構
	Object();
	Object(btCollisionShape* pShape, const btVector3& color, const Eigen::Matrix4d& transMat);
	// 解構
	~Object();

	// accessors
	// 回傳bullet 形狀類
	btCollisionShape* getShape();
	// 回傳bullet 物體類
	btCollisionObject* getCollisionObject() const;
	// 回傳顏色 畫openGL用
	const btVector3& getColor();
	// 設定顏色
	void setColor(const btVector3& color);

	// this transform for gl draw
	void GetTransform(btScalar* transform);
	// 得到物體原心位置
	Eigen::Vector3d getCOM3D() const;
	void setCOM3D(const Eigen::Vector3d& pos);
	Eigen::Matrix4d getCOM6D();
	// 得到物體原心姿態(position+orientation)
	void setCOM6D(const Eigen::Matrix4d& posMat);
	// 設定是否碰撞 用在CheckForCollisionEvents()裡面或reset
	void setContact(bool flag);
	// 回傳是否碰撞
	bool getContact();
	// 得到AABB
	void getaabb(Eigen::Vector3d& aabbMin, Eigen::Vector3d& aabbMax);
	// 設定是否暫時刪除
	void setTempDeleteFlag(bool flag);
	// 得知是否暫時刪除
	bool getTempDeleteFlag();

protected:
	btCollisionShape* m_pShape;
	btCollisionObject* m_pObject;
	btVector3 m_color;
	bool m_contact;
	bool _tempDeleteFlag = false;
};
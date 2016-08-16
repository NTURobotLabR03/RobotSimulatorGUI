/*
//類別名稱：ContactInfo
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：將碰撞的資訊存下來的類(碰撞點、碰撞方向(兩個法向量都有可能)、哪兩個東西撞到)
//使用函式庫：Eigen, Bullet
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
	// 回傳碰撞點
	const btVector3& getPoint() const;
	// 回傳碰撞方向
	const btVector3& getNormal() const;
	// 如果方向錯了可能要call這個function
	void setNormalInv(); // 不知道會不會用到
	// 回傳兩個碰撞物體
	const CollisionPair& getCollisionPair();
	// 碰撞距離(模擬中碰撞是兩個mesh過近) 負號代表已經mesh滲透另外一個物體
	double getDistance();
	// 畫出碰撞點跟方向
	void draw();

protected:
	CollisionPair _pair;
	btVector3 _point;
	btVector3 _normal;
	double _distance;
};


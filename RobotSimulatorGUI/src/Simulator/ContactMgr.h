/*
//類別名稱：ContactMgr
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：將所有碰撞產生的ContactInfo存起來
//使用函式庫：Eigen, Bullet
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
	// push一個ContactInfo
	void push_back(ContactInfo* info);
	// clear掉
	void clear();
	// 回傳有多少ContactInfo
	size_t size();
	// operatorp[] 回傳第i個ContactInfo指標
	ContactInfo* operator[](int i);
	// 設定要不要畫在openGL上
	bool getDebugDrawFlag();
	void setDebugDrawFlag(bool flag);
	// all draw
	void draw();
	// 對某個object 分析
	void analysis(const Object* Object);

protected:
	std::vector<ContactInfo*> _contactInfos;
	bool _debugDrawFlag = false;
	double _eps = -1.0;
};
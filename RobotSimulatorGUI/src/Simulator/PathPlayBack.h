/*
//類別名稱：PathPlayBack
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：用來回放機器跟物體軌跡用的
//使用函式庫：Eigen
*/
#pragma once
#include "EigenHeader.h"
#include "HandArmState.h"
#include "Object.h"

class PathPlayBack{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PathPlayBack();
	// 在建構子給予初始跟最終HandArmState並且記錄中間軌跡，如果有抓握東西 設定object的指標 跟紀錄抓握時 物體到手臂end effector的transformation matrix
	PathPlayBack(const HandArmState& Qinit, const HandArmState& Qgoal, const std::vector<Eigen::VectorXd> path, Object* obj = 0, Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity());
	~PathPlayBack();
	// 查詢軌跡
	const std::vector<Eigen::VectorXd>& getPath() const;
	// 查詢初始狀態
	const HandArmState& getQinit() const;
	// 查詢最終狀態
	const HandArmState& getQgoal() const;
	// 查詢object與end effector的transformation matrix
	const Eigen::Matrix4d& getObjT0() const;
	// 拿取抓取物體的指標
	Object* getObj();

private:
	std::vector<Eigen::VectorXd> _path;
	HandArmState _Qinit, _Qgoal;
	Eigen::Matrix4d _T0;
	Object* _obj;
};
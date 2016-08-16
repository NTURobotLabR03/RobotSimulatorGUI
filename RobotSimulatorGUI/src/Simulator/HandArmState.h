/*
//類別名稱：HandArmState
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：描述Hand跟Arm現在狀態
//使用函式庫：無
*/
#pragma once
#include <vector>

class HandArmState{
public:
	HandArmState();
	~HandArmState();
	// 可以設定 _armTh
	std::vector<double>& getArmTh();
	// 可以查詢 _armTh
	const std::vector<double>& getArmTh() const;
	// 可以設定 _handTh
	std::vector<double>& getHandTh();
	// 可以查詢 _handTh
	const std::vector<double>& HandArmState::getHandTh() const;
	// 設定此狀態是否手掌為抓握狀態
	void setIsGrasp(bool flag);
	// 查詢此狀態是否手掌為抓握狀態
	bool getIsGrasp() const;
private:
	std::vector<double> _armTh, _handTh;
	bool _isGrasp = false;
};
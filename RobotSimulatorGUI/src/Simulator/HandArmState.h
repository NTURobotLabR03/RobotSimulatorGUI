/*
//���O�W�١GHandArmState
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�y�zHand��Arm�{�b���A
//�ϥΨ禡�w�G�L
*/
#pragma once
#include <vector>

class HandArmState{
public:
	HandArmState();
	~HandArmState();
	// �i�H�]�w _armTh
	std::vector<double>& getArmTh();
	// �i�H�d�� _armTh
	const std::vector<double>& getArmTh() const;
	// �i�H�]�w _handTh
	std::vector<double>& getHandTh();
	// �i�H�d�� _handTh
	const std::vector<double>& HandArmState::getHandTh() const;
	// �]�w�����A�O�_��x���촤���A
	void setIsGrasp(bool flag);
	// �d�ߦ����A�O�_��x���촤���A
	bool getIsGrasp() const;
private:
	std::vector<double> _armTh, _handTh;
	bool _isGrasp = false;
};
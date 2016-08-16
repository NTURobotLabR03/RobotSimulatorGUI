/*
//���O�W�١GWrenchGenerator
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�N�Ҧ�Contact�H�ΥؼЪ������L�o�A�M��Q��FrictionCone���h���͹������XWrench���ͪ�Space���e�m�@�~
//�ϥΨ禡�w�GBullet, Eigen
*/
#pragma once
#include "Object.h"
#include "FrictionCone.h"
#include "ContactMgr.h"

class WrenchGenerator{
public:
	// �b�غc�l�o�� �����n�쪺target object, �� contact managerm, friction cone�����]
	WrenchGenerator(Object* object = 0, ContactMgr* contactMgr = 0, FrictionCone* frictionCone = 0);
	~WrenchGenerator();
	// �^�� minkowskiSum �o�ӫܺC �榡�|���GraspWrenchSpaceQualityMeasure��compute
	void minkowskiSum(std::vector<std::vector<double> >& wrenchVectorMinkowskiSum); //�o�Ӥ��n��
	// �^�� union �o�Ӥ���n�� �榡�|���GraspWrenchSpaceQualityMeasure��compute
	void Union(std::vector<std::vector<double> >& wrenchVector);

private:
	double _tauMax = 0.0;
	std::vector<std::vector<Eigen::VectorXd> > _wrenchs; //wij
};
/*
//���O�W�١GGraspWrenchSpaceQualityMeasure
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�NWrenchGenerator�ͥX�Ӫ�Wrench�h��XSpace(Convex Hull), �b�p��X���Quality Measure
//�ϥΨ禡�w�GQhull
*/
#pragma once
#include "QhullHeader.h"
#include <vector>
#include <climits>

class GraspWrenchSpaceQualityMeasure{
public:
	GraspWrenchSpaceQualityMeasure();
	~GraspWrenchSpaceQualityMeasure();

	// �NWrenchGenerator output���榡��i�� �p��convex hull
	bool compute(const std::vector<std::vector<double> >& vertices);
	// �^��Q1 quality measure (�`��)
	double getEps() const;
	// �^��Q2 quality measure (�֥�)
	double getVolume() const;

private:
	orgQhull::Qhull _qhull;
	double _eps = -DBL_MAX;
	double _v = 0.0;
};
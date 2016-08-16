/*
//類別名稱：GraspWrenchSpaceQualityMeasure
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：將WrenchGenerator生出來的Wrench去算出Space(Convex Hull), 在計算出兩種Quality Measure
//使用函式庫：Qhull
*/
#pragma once
#include "QhullHeader.h"
#include <vector>
#include <climits>

class GraspWrenchSpaceQualityMeasure{
public:
	GraspWrenchSpaceQualityMeasure();
	~GraspWrenchSpaceQualityMeasure();

	// 將WrenchGenerator output的格式丟進來 計算convex hull
	bool compute(const std::vector<std::vector<double> >& vertices);
	// 回傳Q1 quality measure (常用)
	double getEps() const;
	// 回傳Q2 quality measure (少用)
	double getVolume() const;

private:
	orgQhull::Qhull _qhull;
	double _eps = -DBL_MAX;
	double _v = 0.0;
};
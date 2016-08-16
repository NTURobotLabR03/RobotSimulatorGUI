/*
//類別名稱：WrenchGenerator
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：將所有Contact以及目標物先做過濾，然後利用FrictionCone類去產生對應的合Wrench產生的Space的前置作業
//使用函式庫：Bullet, Eigen
*/
#pragma once
#include "Object.h"
#include "FrictionCone.h"
#include "ContactMgr.h"

class WrenchGenerator{
public:
	// 在建構子這邊 給予要抓的target object, 跟 contact managerm, friction cone的假設
	WrenchGenerator(Object* object = 0, ContactMgr* contactMgr = 0, FrictionCone* frictionCone = 0);
	~WrenchGenerator();
	// 回傳 minkowskiSum 這個很慢 格式會對到GraspWrenchSpaceQualityMeasure的compute
	void minkowskiSum(std::vector<std::vector<double> >& wrenchVectorMinkowskiSum); //這個不好用
	// 回傳 union 這個比較好用 格式會對到GraspWrenchSpaceQualityMeasure的compute
	void Union(std::vector<std::vector<double> >& wrenchVector);

private:
	double _tauMax = 0.0;
	std::vector<std::vector<Eigen::VectorXd> > _wrenchs; //wij
};
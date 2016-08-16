/*
//類別名稱：FrictionCone
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：產生FrictionCone樣本向量
//使用函式庫：Eigen, Bullet
*/
#pragma once
#include <vector>
#include "EigenHeader.h"
#include "ContactInfo.h"

class FrictionCone{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FrictionCone(int coneSamples = 8, double frictionCoeff = 0.25, double unitForce = 1.0);
	~FrictionCone();
	
	// get
	double getUnitForce();
	double getFrictionCoeff();
	double getFrictionConeAngle();
	double getFrictionConeRad();
	double getFrictionConeHeight();
	int getFrictionConeSamples();

	// set
	void setUnitForce(double unitForce);
	void setFrictionCoeff(double frictionCoeff);
	void setFrctionConeSamples(int coneSamples);

	// important
	void computeConePoints(const ContactInfo& point, std::vector<Eigen::Vector3d>& approximatedFrictionCone);

private:
	//Friction cone relevant parameters
	double _unitForce;
	double _frictionCoeff;
	double _frictionConeAngle;
	double _frictionConeRad;
	double _frictionConeHeight;
	std::vector<Eigen::Vector3d> _frictionConeRimPoints;
	int _frictionConeSamples;
};
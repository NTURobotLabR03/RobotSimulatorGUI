#include "stdafx.h"
#include "FrictionCone.h"
using namespace Eigen;

FrictionCone::FrictionCone(int coneSamples, double frictionCoeff, double unitForce){
	_unitForce = unitForce;
	_frictionCoeff = frictionCoeff;
	_frictionConeSamples = coneSamples;

	_frictionConeAngle = atan(_frictionCoeff);
	_frictionConeRad = unitForce * sin(_frictionConeAngle);
	_frictionConeHeight = unitForce * cos(_frictionConeAngle);

	for (int i = 0; i < _frictionConeSamples; ++i){
		Eigen::Vector3d fi;
		fi(0) = _frictionConeRad * cos(i*2.0*M_PI / _frictionConeSamples); //x
		fi(1) = _frictionConeRad * sin(i*2.0*M_PI / _frictionConeSamples); //y
		fi(2) = _frictionConeHeight; //z
		_frictionConeRimPoints.push_back(fi);
	}
}

FrictionCone::~FrictionCone(){}

double FrictionCone::getUnitForce(){ return _unitForce; }
double FrictionCone::getFrictionCoeff(){ return _frictionCoeff; }
double FrictionCone::getFrictionConeAngle(){ return _frictionConeAngle; }
double FrictionCone::getFrictionConeRad(){ return _frictionConeRad; }
double FrictionCone::getFrictionConeHeight(){ return _frictionConeHeight; }
int FrictionCone::getFrictionConeSamples(){ return _frictionConeSamples; }

void FrictionCone::setUnitForce(double unitForce){
	_frictionConeRimPoints.clear();
	_unitForce = unitForce;
	_frictionConeRad = unitForce * sin(_frictionConeAngle);
	_frictionConeHeight = unitForce * cos(_frictionConeAngle);
	for (int i = 0; i < _frictionConeSamples; ++i){
		Eigen::Vector3d fi;
		fi(0) = _frictionConeRad * cos(i*2.0*M_PI / _frictionConeSamples); //x
		fi(1) = _frictionConeRad * sin(i*2.0*M_PI / _frictionConeSamples); //y
		fi(2) = _frictionConeHeight; //z
		_frictionConeRimPoints.push_back(fi);
	}
}

void FrictionCone::setFrictionCoeff(double frictionCoeff){
	_frictionConeRimPoints.clear();
	_frictionCoeff = frictionCoeff;
	_frictionConeAngle = atan(_frictionCoeff);
	_frictionConeRad = _unitForce * sin(_frictionConeAngle);
	_frictionConeHeight = _unitForce * cos(_frictionConeAngle);
	for (int i = 0; i < _frictionConeSamples; ++i){
		Eigen::Vector3d fi;
		fi(0) = _frictionConeRad * cos(i*2.0*M_PI / _frictionConeSamples); //x
		fi(1) = _frictionConeRad * sin(i*2.0*M_PI / _frictionConeSamples); //y
		fi(2) = _frictionConeHeight; //z
		_frictionConeRimPoints.push_back(fi);
	}
}

void FrictionCone::setFrctionConeSamples(int coneSamples){
	_frictionConeRimPoints.clear();
	_frictionConeSamples = coneSamples;
	for (int i = 0; i < _frictionConeSamples; ++i){
		Eigen::Vector3d fi;
		fi(0) = _frictionConeRad * cos(i*2.0*M_PI / _frictionConeSamples); //x
		fi(1) = _frictionConeRad * sin(i*2.0*M_PI / _frictionConeSamples); //y
		fi(2) = _frictionConeHeight; //z
		_frictionConeRimPoints.push_back(fi);
	}
}

void FrictionCone::computeConePoints(const ContactInfo& point, std::vector<Eigen::Vector3d>& approximatedFrictionCone){
	Vector3d z(0, 0, 1), n;
	Matrix3d R;
	n << -point.getNormal()[0], -point.getNormal()[1], -point.getNormal()[2];
	a2bRotation(z, n, R);

	for (int i = 0; i < _frictionConeSamples; ++i){
		approximatedFrictionCone.push_back(R*_frictionConeRimPoints[i]);
	}
}
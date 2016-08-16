#include "stdafx.h"
#include "EigenHeader.h"
using namespace Eigen;

void kaxisTh2Rot(const Vector3d& o, Matrix3d& R){
	Vector3d ore = o;
	double th = ore.norm();
	if (th > 0.001){
		ore.normalize();
		R = AngleAxisd(th, ore);
	}
	else{
		R = Matrix3d::Identity();
	}
}

void a2bRotation(const Vector3d& a, const Vector3d&b, Matrix3d& R){
	Vector3d z = a;
	Vector3d n = b;
	z.normalize();
	n.normalize();
	double th = acos(z.dot(n));
	if (th < 0.01){
		R = Matrix3d::Identity();
	}
	else if ((M_PI - th) < 0.01){
		R = AngleAxisd(M_PI, Vector3d::UnitY());
	}
	else{
		Vector3d axis;
		axis = z.cross(n);
		axis.normalize();
		R = AngleAxisd(th, axis);
	}
}
/*
Vector3d z(0, 0, 1), n(1, 0, 0);
Matrix3d R;
a2bRotation(z, n, R);
Vector3d p(2, 3, 0), a(1, 0, 2);
cout << R*a + p << endl;
*/

void findPlaneVectors(Eigen::Vector3d& n, std::vector < Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& t){
	t.clear();
	n.normalize();
	t.push_back(Vector3d());
	if (abs(n[2]) < 0.001){
		t[0] << 0, 0, 1.0;
	}
	else{
		t[0] << 1, 1, -(n[0] + n[1]) / n[2];
		t[0].normalize();
	}
	t.push_back(n.cross(t[0]));
	t[1].normalize();
}

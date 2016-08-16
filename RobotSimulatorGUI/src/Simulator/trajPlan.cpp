#include "stdafx.h"
#include "trajPlan.h"
using namespace tp;
using namespace std;
using namespace Eigen;

Eigen::MatrixXd tp::Splines212(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::MatrixXd& qm, double t_total, int select)
{
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	end position : qf
	//	end velocity : qfd
	//	via point : qm
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//
	double T, t;
	int num212Splines, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();

	num212Splines = qm.rows() + 1;
	T = t_total / (3 * num212Splines); //一小段時間


	B = Eigen::MatrixXd::Zero(8 * num212Splines, size); //Ax=b

	//起終點 constraints (position~jerk)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(8 * num212Splines - 2) = qf;
	B.row(8 * num212Splines - 1) = qfd;

	//via points constraints (position)
	if (num212Splines >= 2){
		for (int i = 1; i < num212Splines; i++){
			B.row(8 * i - 2) = qm.row(i - 1);
			B.row(8 * i - 2 + 1) = qm.row(i - 1);
		}
	}

	A = Eigen::MatrixXd::Zero(8 * num212Splines, 8 * num212Splines);
	//% 起終點 constraints (position~velocity)
	A(0, 0) = 1;
	A(1, 1) = 1;
	Eigen::MatrixXd m(1, 3);
	m << 1, T, pow(T, 2);
	A.block<1, 3>(8 * num212Splines - 2, 8 * num212Splines - 3) = m;

	m << 0, 1, 2 * T;
	A.block<1, 3>(8 * num212Splines - 1, 8 * num212Splines - 3) = m;

	//2-1-2 各polynomial之間的連續性, C^1 continuous
	Eigen::MatrixXd m1(1, 5);
	Eigen::MatrixXd m2(1, 5);
	Eigen::MatrixXd m3(1, 5);
	Eigen::MatrixXd m4(1, 5);
	m1 << 1, T, pow(T, 2), -1, 0;
	m2 << 0, 1, 2 * T, 0, -1;
	m3 << 1, T, -1, 0, 0;
	m4 << 0, 1, 0, -1, 0;
	for (int i = 1; i <= num212Splines; i++){
		// between the first 2-polynomial and the second 1-polynomial
		// position continuous
		A.block<1, 5>(8 * i - 6, (i - 1) * 8) = m1;
		// velocity continuous
		A.block<1, 5>(8 * i - 6 + 1, (i - 1) * 8) = m2;
		// between the second 1-polynomial and the third 2-polynomial
		// position continuous
		A.block<1, 5>(8 * i - 6 + 2, (i - 1) * 8 + 3) = m3;
		// velocity continuous
		A.block<1, 5>(8 * i - 6 + 3, (i - 1) * 8 + 3) = m4;
	}

	//via points constraints, C^2 continuous
	Eigen::MatrixXd m5(1, 6);
	Eigen::MatrixXd m6(1, 6);
	Eigen::MatrixXd m7(1, 6);
	Eigen::MatrixXd m8(1, 6);
	m5 << 1, T, pow(T, 2), 0, 0, 0;
	m6 << 0, 0, 0, 1, 0, 0;
	m7 << 0, 1, 2 * T, 0, -1, 0;
	m8 << 0, 0, 2, 0, 0, -2;
	if (num212Splines >= 2){ // 有via points
		for (int i = 1; i< num212Splines; i++){
			// position constraints
			A.block<1, 6>(8 * i - 2, 8 * i - 3) = m5;
			// position constraints
			A.block<1, 6>(8 * i - 2 + 1, 8 * i - 3) = m6;
			// velocity continuous
			A.block<1, 6>(8 * i - 2 + 2, 8 * i - 3) = m7;
			// acceleratoin continuous
			A.block<1, 6>(8 * i - 2 + 3, 8 * i - 3) = m8;
		}
	}

	X = Eigen::MatrixXd::Zero(8 * num212Splines, size); //Ax=b
	X = A.inverse()*B;

	for (int i = 0; i < 3 * num212Splines; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < num212Splines; i++){
		C[3 * i].resize(3, size);
		C[3 * i + 1].resize(2, size);
		C[3 * i + 2].resize(3, size);
	}
	for (int i = 1; i <= num212Splines; i++){
		C[(i - 1) * 3] << X.row((i - 1) * 8), X.row((i - 1) * 8 + 1), X.row((i - 1) * 8 + 2);
		C[(i - 1) * 3 + 1] << X.row((i - 1) * 8 + 3), X.row((i - 1) * 8 + 4);
		C[(i - 1) * 3 + 2] << X.row((i - 1) * 8 + 5), X.row((i - 1) * 8 + 6), X.row((i - 1) * 8 + 7);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration;
	position = Eigen::MatrixXd::Zero(N * 3 * num212Splines + 1, size);
	velocity = Eigen::MatrixXd::Zero(N * 3 * num212Splines + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N * 3 * num212Splines + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= num212Splines; i++){
		for (int j = 0; j <= N; j++){
			// first 2nd-polynomial
			position.row(3 * N*(i - 1) + j) = C[(i - 1) * 3].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3].row(1)*TimeArray(j, 0) + C[(i - 1) * 3].row(0); // position
			velocity.row(3 * N*(i - 1) + j) = 2 * C[(i - 1) * 3].row(2)*TimeArray(j, 0) + C[(i - 1) * 3].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + j) = 2 * C[(i - 1) * 3].row(2); // acceralation
			// second 1st-polynomial
			position.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(1); // velocity
			//acceleration = 0; // acceralation
			// third 2nd-polynomial
			position.row(3 * N*(i - 1) + N * 2 + j) = C[(i - 1) * 3 + 2].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 2].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 2 + j) = 2 * C[(i - 1) * 3 + 2].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 2 + j) = 2 * C[(i - 1) * 3 + 2].row(2); // acceralation
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	}
}

Eigen::MatrixXd tp::Splines212(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, double t_total, int select){
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	end position : qf
	//	end velocity : qfd
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//
	double T, t;
	int num212Splines, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();

	num212Splines = 1;
	T = t_total / (3 * num212Splines); //一小段時間


	B = Eigen::MatrixXd::Zero(8 * num212Splines, size); //Ax=b

	//起終點 constraints (position~jerk)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(8 * num212Splines - 2) = qf;
	B.row(8 * num212Splines - 1) = qfd;

	A = Eigen::MatrixXd::Zero(8 * num212Splines, 8 * num212Splines);
	// 起終點 constraints (position~velocity)
	A(0, 0) = 1;
	A(1, 1) = 1;
	Eigen::MatrixXd m(1, 3);
	m << 1, T, pow(T, 2);
	A.block<1, 3>(8 * num212Splines - 2, 8 * num212Splines - 3) = m;

	m << 0, 1, 2 * T;
	A.block<1, 3>(8 * num212Splines - 1, 8 * num212Splines - 3) = m;

	//2-1-2 各polynomial之間的連續性, C^1 continuous
	Eigen::MatrixXd m1(1, 5);
	Eigen::MatrixXd m2(1, 5);
	Eigen::MatrixXd m3(1, 5);
	Eigen::MatrixXd m4(1, 5);
	m1 << 1, T, pow(T, 2), -1, 0;
	m2 << 0, 1, 2 * T, 0, -1;
	m3 << 1, T, -1, 0, 0;
	m4 << 0, 1, 0, -1, 0;
	for (int i = 1; i <= num212Splines; i++){
		// between the first 2-polynomial and the second 1-polynomial
		// position continuous
		A.block<1, 5>(8 * i - 6, (i - 1) * 8) = m1;
		// velocity continuous
		A.block<1, 5>(8 * i - 6 + 1, (i - 1) * 8) = m2;
		// between the second 1-polynomial and the third 2-polynomial
		// position continuous
		A.block<1, 5>(8 * i - 6 + 2, (i - 1) * 8 + 3) = m3;
		// velocity continuous
		A.block<1, 5>(8 * i - 6 + 3, (i - 1) * 8 + 3) = m4;
	}

	X = Eigen::MatrixXd::Zero(8 * num212Splines, size); //Ax=b
	X = A.inverse()*B;

	for (int i = 0; i < 3 * num212Splines; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < num212Splines; i++){
		C[3 * i].resize(3, size);
		C[3 * i + 1].resize(2, size);
		C[3 * i + 2].resize(3, size);
	}
	for (int i = 1; i <= num212Splines; i++){
		C[(i - 1) * 3] << X.row((i - 1) * 8), X.row((i - 1) * 8 + 1), X.row((i - 1) * 8 + 2);
		C[(i - 1) * 3 + 1] << X.row((i - 1) * 8 + 3), X.row((i - 1) * 8 + 4);
		C[(i - 1) * 3 + 2] << X.row((i - 1) * 8 + 5), X.row((i - 1) * 8 + 6), X.row((i - 1) * 8 + 7);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration;
	position = Eigen::MatrixXd::Zero(N * 3 * num212Splines + 1, size);
	velocity = Eigen::MatrixXd::Zero(N * 3 * num212Splines + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N * 3 * num212Splines + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= num212Splines; i++){
		for (int j = 0; j <= N; j++){
			// first 2nd-polynomial
			position.row(3 * N*(i - 1) + j) = C[(i - 1) * 3].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3].row(1)*TimeArray(j, 0) + C[(i - 1) * 3].row(0); // position
			velocity.row(3 * N*(i - 1) + j) = 2 * C[(i - 1) * 3].row(2)*TimeArray(j, 0) + C[(i - 1) * 3].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + j) = 2 * C[(i - 1) * 3].row(2); // acceralation
			// second 1st-polynomial
			position.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(1); // velocity
			//acceleration = 0; // acceralation
			// third 2nd-polynomial
			position.row(3 * N*(i - 1) + N * 2 + j) = C[(i - 1) * 3 + 2].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 2].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 2 + j) = 2 * C[(i - 1) * 3 + 2].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 2 + j) = 2 * C[(i - 1) * 3 + 2].row(2); // acceralation
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	default:
		return Eigen::MatrixXd();
		break;
	}
}

Eigen::MatrixXd tp::Splines434(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, const Eigen::MatrixXd& qm, double t_total, int select)
{
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	initial acceleration : q0dd
	//	end position : qf
	//	end velocity : qfd
	//	end acceleration : qfdd
	//	via point : qm
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//

	double T, t;
	int num434Splines, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();
	Eigen::MatrixXd qmd(qm.rows(), qm.cols());

	num434Splines = qm.rows() + 1;
	T = t_total / (3 * num434Splines); //一小段時間

	Eigen::MatrixXd Q;
	Q = Eigen::MatrixXd::Zero(qm.rows() + 2, size); //Ax=b
	Q.row(0) = q0;
	for (int i = 0; i < qm.rows(); ++i)
		Q.row(i + 1) = qm.row(i);
	Q.row(qm.rows() + 1) = qf;
	// 決定via points的速度
	if (num434Splines >= 2){
		for (int i = 1; i < num434Splines; i++){
			for (int j = 0; j < size; j++){
				if ((Q(i, j) - Q(i - 1, j)) * (Q(i + 1, j) - Q(i, j)) > 0)
					qmd(i - 1, j) = ((Q(i, j) - Q(i - 1, j)) / (3 * T) + (Q(i + 1, j) - Q(i, j)) / (3 * T)) / 2;
				else
					qmd(i - 1, j) = 0;
			}
		}
	}

	B = Eigen::MatrixXd::Zero(14 * num434Splines, size); //Ax=b

	//起終點 constraints (position~a)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(2) = q0dd;
	B.row(14 * num434Splines - 3) = qf;
	B.row(14 * num434Splines - 2) = qfd;
	B.row(14 * num434Splines - 1) = qfdd;

	//via points constraints (position & velocity)
	if (num434Splines >= 2){
		for (int i = 1; i < num434Splines; i++){
			B.row(14 * i - 3) = qm.row(i - 1);
			B.row(14 * i - 3 + 1) = qmd.row(i - 1);
			B.row(14 * i - 3 + 2) = qm.row(i - 1);
			B.row(14 * i - 3 + 3) = qmd.row(i - 1);
		}
	}

	A = Eigen::MatrixXd::Zero(14 * num434Splines, 14 * num434Splines);
	//% 起終點 constraints (position~jerk)
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(2, 2) = 2;
	Eigen::MatrixXd m(1, 5);
	m << 1, T, pow(T, 2), pow(T, 3), pow(T, 4);
	A.block<1, 5>(14 * num434Splines - 3, 14 * num434Splines - 5) = m;

	m << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3);
	A.block<1, 5>(14 * num434Splines - 2, 14 * num434Splines - 5) = m;

	m << 0, 0, 2, 6 * T, 12 * pow(T, 2);
	A.block<1, 5>(14 * num434Splines - 1, 14 * num434Splines - 5) = m;

	//4-3-4 各polynomial之間的連續性, C^3 continuous
	Eigen::MatrixXd m1(1, 9);
	Eigen::MatrixXd m2(1, 9);
	Eigen::MatrixXd m3(1, 9);
	Eigen::MatrixXd m4(1, 9);
	Eigen::MatrixXd m5(1, 9);
	Eigen::MatrixXd m6(1, 9);
	Eigen::MatrixXd m7(1, 9);
	Eigen::MatrixXd m8(1, 9);

	m1 << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), -1, 0, 0, 0;
	m2 << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 0, -1, 0, 0;
	m3 << 0, 0, 2, 6 * T, 12 * pow(T, 2), 0, 0, -2, 0;
	m4 << 0, 0, 0, 6, 24 * T, 0, 0, 0, -6;
	m5 << 1, T, pow(T, 2), pow(T, 3), -1, 0, 0, 0, 0;
	m6 << 0, 1, 2 * T, 3 * pow(T, 2), 0, -1, 0, 0, 0;
	m7 << 0, 0, 2, 6 * T, 0, 0, -2, 0, 0;
	m8 << 0, 0, 0, 6, 0, 0, 0, -6, 0;
	for (int i = 1; i <= num434Splines; i++){
		// between the first 4-polynomial and the second 3-polynomial
		// position continuous
		A.block<1, 9>(14 * i - 11, (i - 1) * 14) = m1;
		// velocity continuous
		A.block<1, 9>(14 * i - 11 + 1, (i - 1) * 14) = m2;
		// acceleration continuous
		A.block<1, 9>(14 * i - 11 + 2, (i - 1) * 14) = m3;
		// jerk continuous
		A.block<1, 9>(14 * i - 11 + 3, (i - 1) * 14) = m4;

		// between the second 3-polynomial and the third 4-polynomial
		// position continuous
		A.block<1, 9>(14 * i - 11 + 4, 14 * i - 9) = m5;
		// velocity continuous
		A.block<1, 9>(14 * i - 11 + 5, 14 * i - 9) = m6;
		// acceleration continuous
		A.block<1, 9>(14 * i - 11 + 6, 14 * i - 9) = m7;
		// jerk continuous
		A.block<1, 9>(14 * i - 11 + 7, 14 * i - 9) = m8;
	}

	// via points constraints, C^3 continuous
	Eigen::MatrixXd m9(1, 10);
	Eigen::MatrixXd m10(1, 10);
	Eigen::MatrixXd m11(1, 10);
	Eigen::MatrixXd m12(1, 10);
	Eigen::MatrixXd m13(1, 10);
	Eigen::MatrixXd m14(1, 10);

	m9 << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), 0, 0, 0, 0, 0;
	m10 << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 0, 0, 0, 0, 0;
	m11 << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
	m12 << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
	m13 << 0, 0, 2, 6 * T, 12 * pow(T, 2), 0, 0, -2, 0, 0;
	m14 << 0, 0, 0, 6, 24 * T, 0, 0, 0, -6, 0;
	if (num434Splines >= 2){ // 有via points
		for (int i = 1; i< num434Splines; i++){
			// position constraints
			A.block<1, 10>(14 * i - 3, 14 * i - 5) = m9;
			// position constraints
			A.block<1, 10>(14 * i - 3 + 1, 14 * i - 5) = m10;
			// velocity continuous
			A.block<1, 10>(14 * i - 3 + 2, 14 * i - 5) = m11;
			// acceleratoin continuous
			A.block<1, 10>(14 * i - 3 + 3, 14 * i - 5) = m12;


			// acceleration continuous
			A.block<1, 10>(14 * i - 3 + 4, 14 * i - 5) = m13;
			// jerk continuous
			A.block<1, 10>(14 * i - 3 + 5, 14 * i - 5) = m14;
		}
	}

	X = Eigen::MatrixXd::Zero(14 * num434Splines, size); //Ax=b
	X = A.inverse()*B;


	for (int i = 0; i < 3 * num434Splines; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < num434Splines; i++){
		C[3 * i].resize(5, size);
		C[3 * i + 1].resize(4, size);
		C[3 * i + 2].resize(5, size);
	}
	for (int i = 1; i <= num434Splines; i++){
		C[(i - 1) * 3] << X.row((i - 1) * 14), X.row((i - 1) * 14 + 1), X.row((i - 1) * 14 + 2), X.row((i - 1) * 14 + 3), X.row((i - 1) * 14 + 4);
		C[(i - 1) * 3 + 1] << X.row((i - 1) * 14 + 5), X.row((i - 1) * 14 + 6), X.row((i - 1) * 14 + 7), X.row((i - 1) * 14 + 8);
		C[(i - 1) * 3 + 2] << X.row((i - 1) * 14 + 9), X.row((i - 1) * 14 + 10), X.row((i - 1) * 14 + 11), X.row((i - 1) * 14 + 12), X.row((i - 1) * 14 + 13);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration, jerk;
	position = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);
	velocity = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);
	jerk = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= num434Splines; i++){
		for (int j = 0; j <= N; j++){
			// first 4th-polynomial
			position.row(3 * N*(i - 1) + j) = C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3].row(1)*TimeArray(j, 0) + C[(i - 1) * 3].row(0); // position
			velocity.row(3 * N*(i - 1) + j) = 4 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3].row(2)*TimeArray(j, 0) + C[(i - 1) * 3].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + j) = 12 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + j) = 24 * C[(i - 1) * 3].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3].row(3);// jerk
			// second 3th-polynomial
			position.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 1].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 1].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 1 + j) = 3 * C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 1].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 1].row(2); // velocity; // acceralation
			jerk.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3);// jerk
			// third 4th-polynomial
			position.row(3 * N*(i - 1) + N * 2 + j) = C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 2].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 2].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 2 + j) = 4 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 2].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 2 + j) = 12 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3 + 2].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 2].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + N * 2 + j) = 24 * C[(i - 1) * 3 + 2].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3 + 2].row(3);// jerk
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	}
}

Eigen::MatrixXd tp::Splines434(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, double t_total, int select)
{
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	initial acceleration : q0dd
	//	end position : qf
	//	end velocity : qfd
	//	end acceleration : qfdd
	//	via point : qm
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//
	double T, t;
	int num434Splines, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();

	num434Splines = 1;
	T = t_total / (3 * num434Splines); //一小段時間

	Eigen::MatrixXd Q;
	Q = Eigen::MatrixXd::Zero(2, size); //Ax=b
	Q.row(0) = q0;
	Q.row(1) = qf;

	B = Eigen::MatrixXd::Zero(14 * num434Splines, size); //Ax=b

	//起終點 constraints (position~a)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(2) = q0dd;
	B.row(14 * num434Splines - 3) = qf;
	B.row(14 * num434Splines - 2) = qfd;
	B.row(14 * num434Splines - 1) = qfdd;


	A = Eigen::MatrixXd::Zero(14 * num434Splines, 14 * num434Splines);
	//% 起終點 constraints (position~jerk)
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(2, 2) = 2;
	Eigen::MatrixXd m(1, 5);
	m << 1, T, pow(T, 2), pow(T, 3), pow(T, 4);
	A.block<1, 5>(14 * num434Splines - 3, 14 * num434Splines - 5) = m;

	m << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3);
	A.block<1, 5>(14 * num434Splines - 2, 14 * num434Splines - 5) = m;

	m << 0, 0, 2, 6 * T, 12 * pow(T, 2);
	A.block<1, 5>(14 * num434Splines - 1, 14 * num434Splines - 5) = m;

	//4-3-4 各polynomial之間的連續性, C^3 continuous
	Eigen::MatrixXd m1(1, 9);
	Eigen::MatrixXd m2(1, 9);
	Eigen::MatrixXd m3(1, 9);
	Eigen::MatrixXd m4(1, 9);
	Eigen::MatrixXd m5(1, 9);
	Eigen::MatrixXd m6(1, 9);
	Eigen::MatrixXd m7(1, 9);
	Eigen::MatrixXd m8(1, 9);

	m1 << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), -1, 0, 0, 0;
	m2 << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 0, -1, 0, 0;
	m3 << 0, 0, 2, 6 * T, 12 * pow(T, 2), 0, 0, -2, 0;
	m4 << 0, 0, 0, 6, 24 * T, 0, 0, 0, -6;
	m5 << 1, T, pow(T, 2), pow(T, 3), -1, 0, 0, 0, 0;
	m6 << 0, 1, 2 * T, 3 * pow(T, 2), 0, -1, 0, 0, 0;
	m7 << 0, 0, 2, 6 * T, 0, 0, -2, 0, 0;
	m8 << 0, 0, 0, 6, 0, 0, 0, -6, 0;
	for (int i = 1; i <= num434Splines; i++){
		// between the first 4-polynomial and the second 3-polynomial
		// position continuous
		A.block<1, 9>(14 * i - 11, (i - 1) * 14) = m1;
		// velocity continuous
		A.block<1, 9>(14 * i - 11 + 1, (i - 1) * 14) = m2;
		// acceleration continuous
		A.block<1, 9>(14 * i - 11 + 2, (i - 1) * 14) = m3;
		// jerk continuous
		A.block<1, 9>(14 * i - 11 + 3, (i - 1) * 14) = m4;

		// between the second 3-polynomial and the third 4-polynomial
		// position continuous
		A.block<1, 9>(14 * i - 11 + 4, 14 * i - 9) = m5;
		// velocity continuous
		A.block<1, 9>(14 * i - 11 + 5, 14 * i - 9) = m6;
		// acceleration continuous
		A.block<1, 9>(14 * i - 11 + 6, 14 * i - 9) = m7;
		// jerk continuous
		A.block<1, 9>(14 * i - 11 + 7, 14 * i - 9) = m8;
	}

	X = Eigen::MatrixXd::Zero(14 * num434Splines, size); //Ax=b
	X = A.inverse()*B;


	for (int i = 0; i < 3 * num434Splines; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < num434Splines; i++){
		C[3 * i].resize(5, size);
		C[3 * i + 1].resize(4, size);
		C[3 * i + 2].resize(5, size);
	}
	for (int i = 1; i <= num434Splines; i++){
		C[(i - 1) * 3] << X.row((i - 1) * 14), X.row((i - 1) * 14 + 1), X.row((i - 1) * 14 + 2), X.row((i - 1) * 14 + 3), X.row((i - 1) * 14 + 4);
		C[(i - 1) * 3 + 1] << X.row((i - 1) * 14 + 5), X.row((i - 1) * 14 + 6), X.row((i - 1) * 14 + 7), X.row((i - 1) * 14 + 8);
		C[(i - 1) * 3 + 2] << X.row((i - 1) * 14 + 9), X.row((i - 1) * 14 + 10), X.row((i - 1) * 14 + 11), X.row((i - 1) * 14 + 12), X.row((i - 1) * 14 + 13);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration, jerk;
	position = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);
	velocity = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);
	jerk = Eigen::MatrixXd::Zero(N * 3 * num434Splines + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= num434Splines; i++){
		for (int j = 0; j <= N; j++){
			// first 4th-polynomial
			position.row(3 * N*(i - 1) + j) = C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3].row(1)*TimeArray(j, 0) + C[(i - 1) * 3].row(0); // position
			velocity.row(3 * N*(i - 1) + j) = 4 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3].row(2)*TimeArray(j, 0) + C[(i - 1) * 3].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + j) = 12 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + j) = 24 * C[(i - 1) * 3].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3].row(3);// jerk
			// second 3th-polynomial
			position.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 1].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 1].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 1 + j) = 3 * C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 1].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 1].row(2); // velocity; // acceralation
			jerk.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3);// jerk
			// third 4th-polynomial
			position.row(3 * N*(i - 1) + N * 2 + j) = C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 2].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 2].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 2 + j) = 4 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 2].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 2 + j) = 12 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3 + 2].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 2].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + N * 2 + j) = 24 * C[(i - 1) * 3 + 2].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3 + 2].row(3);// jerk
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	}
}

Eigen::MatrixXd tp::Splines535(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& q0ddd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, const Eigen::VectorXd& qfddd, const Eigen::MatrixXd& qm, double t_total, int select)
{
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	initial acceleration : q0dd
	//	initial jerk : q0ddd
	//	end position : qf
	//	end velocity : qfd
	//	end acceleration : qfdd
	//	end jerk : qfddd
	//	via point : qm
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//
	double T, t;
	int num535Splines, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();
	Eigen::MatrixXd qmd(qm.rows(), qm.cols());

	num535Splines = qm.rows() + 1;
	T = t_total / (3 * num535Splines); //一小段時間


	Eigen::MatrixXd Q;
	Q = Eigen::MatrixXd::Zero(qm.rows() + 2, size); //Ax=b
	Q.row(0) = q0;
	for (int i = 0; i < qm.rows(); ++i)
		Q.row(i + 1) = qm.row(i);
	Q.row(qm.rows() + 1) = qf;
	// 決定via points的速度
	if (num535Splines >= 2){
		for (int i = 1; i < num535Splines; i++){
			for (int j = 0; j < size; j++){
				if ((Q(i, j) - Q(i - 1, j)) * (Q(i + 1, j) - Q(i, j)) > 0)
					qmd(i - 1, j) = ((Q(i, j) - Q(i - 1, j)) / (3 * T) + (Q(i + 1, j) - Q(i, j)) / (3 * T)) / 2;
				else
					qmd(i - 1, j) = 0;
			}
		}
	}


	B = Eigen::MatrixXd::Zero(16 * num535Splines, size); //Ax=b

	//起終點 constraints (position~a)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(2) = q0dd;
	B.row(3) = q0ddd;
	B.row(16 * num535Splines - 4) = qf;
	B.row(16 * num535Splines - 3) = qfd;
	B.row(16 * num535Splines - 2) = qfdd;
	B.row(16 * num535Splines - 1) = qfddd;


	//via points constraints (position & velocity)
	if (num535Splines >= 2){
		for (int i = 1; i < num535Splines; i++){
			B.row(16 * i - 4) = qm.row(i - 1);
			B.row(16 * i - 4 + 1) = qmd.row(i - 1);
			B.row(16 * i - 4 + 2) = qm.row(i - 1);
			B.row(16 * i - 4 + 3) = qmd.row(i - 1);
		}
	}


	A = Eigen::MatrixXd::Zero(16 * num535Splines, 16 * num535Splines);
	//% 起終點 constraints (position~jerk)
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(2, 2) = 2;
	A(3, 3) = 6;
	Eigen::MatrixXd m(1, 6);
	m << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5);
	A.block<1, 6>(16 * num535Splines - 4, 16 * num535Splines - 6) = m;

	m << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4);
	A.block<1, 6>(16 * num535Splines - 3, 16 * num535Splines - 6) = m;

	m << 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
	A.block<1, 6>(16 * num535Splines - 2, 16 * num535Splines - 6) = m;

	m << 0, 0, 0, 6, 24 * T, 60 * pow(T, 2);
	A.block<1, 6>(16 * num535Splines - 1, 16 * num535Splines - 6) = m;

	// 5-3-5 各polynomial之間的連續性, C^3 continuous
	Eigen::MatrixXd m1(1, 10);
	Eigen::MatrixXd m2(1, 10);
	Eigen::MatrixXd m3(1, 10);
	Eigen::MatrixXd m4(1, 10);
	Eigen::MatrixXd m5(1, 10);
	Eigen::MatrixXd m6(1, 10);
	Eigen::MatrixXd m7(1, 10);
	Eigen::MatrixXd m8(1, 10);

	m1 << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5), -1, 0, 0, 0;
	m2 << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4), 0, -1, 0, 0;
	m3 << 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3), 0, 0, -2, 0;
	m4 << 0, 0, 0, 6, 24 * T, 60 * pow(T, 2), 0, 0, 0, -6;
	m5 << 1, T, pow(T, 2), pow(T, 3), -1, 0, 0, 0, 0, 0;
	m6 << 0, 1, 2 * T, 3 * pow(T, 2), 0, -1, 0, 0, 0, 0;
	m7 << 0, 0, 2, 6 * T, 0, 0, -2, 0, 0, 0;
	m8 << 0, 0, 0, 1, 0, 0, 0, -1, 0, 0;
	for (int i = 1; i <= num535Splines; i++){
		// between the first 5-polynomial and the second 3-polynomial
		// position continuous
		A.block<1, 10>(16 * i - 12, (i - 1) * 16) = m1;
		// velocity continuous
		A.block<1, 10>(16 * i - 12 + 1, (i - 1) * 16) = m2;
		// acceleration continuous
		A.block<1, 10>(16 * i - 12 + 2, (i - 1) * 16) = m3;
		// jerk continuous
		A.block<1, 10>(16 * i - 12 + 3, (i - 1) * 16) = m4;

		// between the second 3-polynomial and the third 5-polynomial
		// position continuous
		A.block<1, 10>(16 * i - 12 + 4, (i - 1) * 16 + 6) = m5;
		// velocity continuous
		A.block<1, 10>(16 * i - 12 + 5, (i - 1) * 16 + 6) = m6;
		// acceleration continuous
		A.block<1, 10>(16 * i - 12 + 6, (i - 1) * 16 + 6) = m7;
		// jerk continuous
		A.block<1, 10>(16 * i - 12 + 7, (i - 1) * 16 + 6) = m8;
	}

	// via points constraints, C^5 continuous
	Eigen::MatrixXd m9(1, 12);
	Eigen::MatrixXd m10(1, 12);
	Eigen::MatrixXd m11(1, 12);
	Eigen::MatrixXd m12(1, 12);
	Eigen::MatrixXd m13(1, 12);
	Eigen::MatrixXd m14(1, 12);
	Eigen::MatrixXd m15(1, 12);
	Eigen::MatrixXd m16(1, 12);

	m9 << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5), 0, 0, 0, 0, 0, 0;
	m10 << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4), 0, 0, 0, 0, 0, 0;
	m11 << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
	m12 << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
	m13 << 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3), 0, 0, -2, 0, 0, 0;
	m14 << 0, 0, 0, 6, 24 * T, 60 * pow(T, 2), 0, 0, 0, -6, 0, 0;
	m15 << 0, 0, 0, 0, 24, 120 * T, 0, 0, 0, 0, -24, 0;
	m16 << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1;
	if (num535Splines >= 2){ // 有via points
		for (int i = 1; i< num535Splines; i++){
			// position constraints
			A.block<1, 12>(16 * i - 4, (i - 1) * 16 + 10) = m9;
			// velocity constraints
			A.block<1, 12>(16 * i - 4 + 1, (i - 1) * 16 + 10) = m10;
			// position constraints
			A.block<1, 12>(16 * i - 4 + 2, (i - 1) * 16 + 10) = m11;
			// velocity constraints
			A.block<1, 12>(16 * i - 4 + 3, (i - 1) * 16 + 10) = m12;


			// acceleration continuous
			A.block<1, 12>(16 * i - 4 + 4, (i - 1) * 16 + 10) = m13;
			// jerk continuous
			A.block<1, 12>(16 * i - 4 + 5, (i - 1) * 16 + 10) = m14;
			// C^4 continuous
			A.block<1, 12>(16 * i - 4 + 6, (i - 1) * 16 + 10) = m15;
			// C^5 continuous
			A.block<1, 12>(16 * i - 4 + 7, (i - 1) * 16 + 10) = m16;
		}
	}

	X = Eigen::MatrixXd::Zero(16 * num535Splines, size); //Ax=b
	X = A.inverse()*B;


	for (int i = 0; i < 3 * num535Splines; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < num535Splines; i++){
		C[3 * i].resize(6, size);
		C[3 * i + 1].resize(4, size);
		C[3 * i + 2].resize(6, size);
	}
	for (int i = 1; i <= num535Splines; i++){
		C[(i - 1) * 3] << X.row((i - 1) * 16), X.row((i - 1) * 16 + 1), X.row((i - 1) * 16 + 2), X.row((i - 1) * 16 + 3), X.row((i - 1) * 16 + 4), X.row((i - 1) * 16 + 5);
		C[(i - 1) * 3 + 1] << X.row((i - 1) * 16 + 6), X.row((i - 1) * 16 + 7), X.row((i - 1) * 16 + 8), X.row((i - 1) * 16 + 9);
		C[(i - 1) * 3 + 2] << X.row((i - 1) * 16 + 10), X.row((i - 1) * 16 + 11), X.row((i - 1) * 16 + 12), X.row((i - 1) * 16 + 13), X.row((i - 1) * 16 + 14), X.row((i - 1) * 16 + 15);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration, jerk;
	position = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);
	velocity = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);
	jerk = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= num535Splines; i++){
		for (int j = 0; j <= N; j++){
			// first 5th-polynomial
			position.row(3 * N*(i - 1) + j) = C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 5) + C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3].row(1)*TimeArray(j, 0) + C[(i - 1) * 3].row(0); // position
			velocity.row(3 * N*(i - 1) + j) = 5 * C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 4) + 4 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3].row(2)*TimeArray(j, 0) + C[(i - 1) * 3].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + j) = 20 * C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 3) + 12 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + j) = 60 * C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 2) + 24 * C[(i - 1) * 3].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3].row(3);// jerk
			// second 3th-polynomial
			position.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 1].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 1].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 1 + j) = 3 * C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 1].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 1].row(2); // velocity; // acceralation
			jerk.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3);// jerk
			// third 5th-polynomial
			position.row(3 * N*(i - 1) + N * 2 + j) = C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 5) + C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 2].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 2].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 2 + j) = 5 * C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 4) + 4 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 2].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 2 + j) = 20 * C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 3) + 12 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3 + 2].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 2].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + N * 2 + j) = 60 * C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 2) + 24 * C[(i - 1) * 3 + 2].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3 + 2].row(3);// jerk
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	}

}

Eigen::MatrixXd tp::Splines535(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& q0ddd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, const Eigen::VectorXd& qfddd, double t_total, int select)
{
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	initial acceleration : q0dd
	//	initial jerk : q0ddd
	//	end position : qf
	//	end velocity : qfd
	//	end acceleration : qfdd
	//	end jerk : qfddd
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//
	double T, t;
	int num535Splines, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();

	num535Splines = 1;
	T = t_total / (3 * num535Splines); //一小段時間


	Eigen::MatrixXd Q;
	Q = Eigen::MatrixXd::Zero(2, size); //Ax=b
	Q.row(0) = q0;
	Q.row(1) = qf;

	B = Eigen::MatrixXd::Zero(16 * num535Splines, size); //Ax=b

	//起終點 constraints (position~a)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(2) = q0dd;
	B.row(3) = q0ddd;
	B.row(16 * num535Splines - 4) = qf;
	B.row(16 * num535Splines - 3) = qfd;
	B.row(16 * num535Splines - 2) = qfdd;
	B.row(16 * num535Splines - 1) = qfddd;

	A = Eigen::MatrixXd::Zero(16 * num535Splines, 16 * num535Splines);
	//% 起終點 constraints (position~jerk)
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(2, 2) = 2;
	A(3, 3) = 6;
	Eigen::MatrixXd m(1, 6);
	m << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5);
	A.block<1, 6>(16 * num535Splines - 4, 16 * num535Splines - 6) = m;

	m << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4);
	A.block<1, 6>(16 * num535Splines - 3, 16 * num535Splines - 6) = m;

	m << 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
	A.block<1, 6>(16 * num535Splines - 2, 16 * num535Splines - 6) = m;

	m << 0, 0, 0, 6, 24 * T, 60 * pow(T, 2);
	A.block<1, 6>(16 * num535Splines - 1, 16 * num535Splines - 6) = m;

	// 5-3-5 各polynomial之間的連續性, C^3 continuous
	Eigen::MatrixXd m1(1, 10);
	Eigen::MatrixXd m2(1, 10);
	Eigen::MatrixXd m3(1, 10);
	Eigen::MatrixXd m4(1, 10);
	Eigen::MatrixXd m5(1, 10);
	Eigen::MatrixXd m6(1, 10);
	Eigen::MatrixXd m7(1, 10);
	Eigen::MatrixXd m8(1, 10);

	m1 << 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5), -1, 0, 0, 0;
	m2 << 0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4), 0, -1, 0, 0;
	m3 << 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3), 0, 0, -2, 0;
	m4 << 0, 0, 0, 6, 24 * T, 60 * pow(T, 2), 0, 0, 0, -6;
	m5 << 1, T, pow(T, 2), pow(T, 3), -1, 0, 0, 0, 0, 0;
	m6 << 0, 1, 2 * T, 3 * pow(T, 2), 0, -1, 0, 0, 0, 0;
	m7 << 0, 0, 2, 6 * T, 0, 0, -2, 0, 0, 0;
	m8 << 0, 0, 0, 1, 0, 0, 0, -1, 0, 0;
	for (int i = 1; i <= num535Splines; i++){
		// between the first 5-polynomial and the second 3-polynomial
		// position continuous
		A.block<1, 10>(16 * i - 12, (i - 1) * 16) = m1;
		// velocity continuous
		A.block<1, 10>(16 * i - 12 + 1, (i - 1) * 16) = m2;
		// acceleration continuous
		A.block<1, 10>(16 * i - 12 + 2, (i - 1) * 16) = m3;
		// jerk continuous
		A.block<1, 10>(16 * i - 12 + 3, (i - 1) * 16) = m4;

		// between the second 3-polynomial and the third 5-polynomial
		// position continuous
		A.block<1, 10>(16 * i - 12 + 4, (i - 1) * 16 + 6) = m5;
		// velocity continuous
		A.block<1, 10>(16 * i - 12 + 5, (i - 1) * 16 + 6) = m6;
		// acceleration continuous
		A.block<1, 10>(16 * i - 12 + 6, (i - 1) * 16 + 6) = m7;
		// jerk continuous
		A.block<1, 10>(16 * i - 12 + 7, (i - 1) * 16 + 6) = m8;
	}

	X = Eigen::MatrixXd::Zero(16 * num535Splines, size); //Ax=b
	X = A.inverse()*B;


	for (int i = 0; i < 3 * num535Splines; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < num535Splines; i++){
		C[3 * i].resize(6, size);
		C[3 * i + 1].resize(4, size);
		C[3 * i + 2].resize(6, size);
	}
	for (int i = 1; i <= num535Splines; i++){
		C[(i - 1) * 3] << X.row((i - 1) * 16), X.row((i - 1) * 16 + 1), X.row((i - 1) * 16 + 2), X.row((i - 1) * 16 + 3), X.row((i - 1) * 16 + 4), X.row((i - 1) * 16 + 5);
		C[(i - 1) * 3 + 1] << X.row((i - 1) * 16 + 6), X.row((i - 1) * 16 + 7), X.row((i - 1) * 16 + 8), X.row((i - 1) * 16 + 9);
		C[(i - 1) * 3 + 2] << X.row((i - 1) * 16 + 10), X.row((i - 1) * 16 + 11), X.row((i - 1) * 16 + 12), X.row((i - 1) * 16 + 13), X.row((i - 1) * 16 + 14), X.row((i - 1) * 16 + 15);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration, jerk;
	position = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);
	velocity = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);
	jerk = Eigen::MatrixXd::Zero(N * 3 * num535Splines + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= num535Splines; i++){
		for (int j = 0; j <= N; j++){
			// first 5th-polynomial
			position.row(3 * N*(i - 1) + j) = C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 5) + C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3].row(1)*TimeArray(j, 0) + C[(i - 1) * 3].row(0); // position
			velocity.row(3 * N*(i - 1) + j) = 5 * C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 4) + 4 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3].row(2)*TimeArray(j, 0) + C[(i - 1) * 3].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + j) = 20 * C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 3) + 12 * C[(i - 1) * 3].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + j) = 60 * C[(i - 1) * 3].row(5)*pow(TimeArray(j, 0), 2) + 24 * C[(i - 1) * 3].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3].row(3);// jerk
			// second 3th-polynomial
			position.row(3 * N*(i - 1) + N * 1 + j) = C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 1].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 1].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 1 + j) = 3 * C[(i - 1) * 3 + 1].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 1].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 1].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 1].row(2); // velocity; // acceralation
			jerk.row(3 * N*(i - 1) + N * 1 + j) = 6 * C[(i - 1) * 3 + 1].row(3);// jerk
			// third 5th-polynomial
			position.row(3 * N*(i - 1) + N * 2 + j) = C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 5) + C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 4) + C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1) * 3 + 2].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1) * 3 + 2].row(1)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(0); // position
			velocity.row(3 * N*(i - 1) + N * 2 + j) = 5 * C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 4) + 4 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 3) + 3 * C[(i - 1) * 3 + 2].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1) * 3 + 2].row(2)*TimeArray(j, 0) + C[(i - 1) * 3 + 2].row(1); // velocity
			acceleration.row(3 * N*(i - 1) + N * 2 + j) = 20 * C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 3) + 12 * C[(i - 1) * 3 + 2].row(4)*pow(TimeArray(j, 0), 2) + 6 * C[(i - 1) * 3 + 2].row(3)*TimeArray(j, 0) + 2 * C[(i - 1) * 3 + 2].row(2);  // acceralation
			jerk.row(3 * N*(i - 1) + N * 2 + j) = 60 * C[(i - 1) * 3 + 2].row(5)*pow(TimeArray(j, 0), 2) + 24 * C[(i - 1) * 3 + 2].row(4)*TimeArray(j, 0) + 6 * C[(i - 1) * 3 + 2].row(3);// jerk
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	}

}

Eigen::MatrixXd tp::CubicPolynomials(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::MatrixXd& qm, double t_total, int select)
{
	//input : 
	//	initial position : q0
	//	initial velocity : q0d
	//	end position : qf
	//	end velocity : qfd
	//	via point : qm
	//	total time : t_total
	//
	//output : 
	//	select :  position : 1   velocity : 2  acceleration : 3  
	//
	double T, t;
	int numCubicPolynomials, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();

	numCubicPolynomials = qm.rows() + 1;

	T = t_total / numCubicPolynomials; //一小段時間

	B = Eigen::MatrixXd::Zero(4 * numCubicPolynomials, size); //Ax=b

	// 起終點 constraints (position, velocity)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(4 * numCubicPolynomials - 2) = qf;
	B.row(4 * numCubicPolynomials - 1) = qfd;

	// via points constraints (position)
	if (numCubicPolynomials >= 2){
		for (int i = 1; i < numCubicPolynomials; i++){
			B.row(4 * i - 2) = qm.row(i - 1);
			B.row(4 * i - 2 + 1) = qm.row(i - 1);
		}
	}

	A = Eigen::MatrixXd::Zero(4 * numCubicPolynomials, 4 * numCubicPolynomials);
	// 起終點 constraints (position, velocity)
	A(0, 0) = 1;
	A(1, 1) = 1;
	Eigen::MatrixXd m(1, 4);
	m << 1, T, pow(T, 2), pow(T, 3);
	A.block<1, 4>(4 * numCubicPolynomials - 2, 4 * numCubicPolynomials - 4) = m;

	m << 0, 1, 2 * T, 3 * pow(T, 2);
	A.block<1, 4>(4 * numCubicPolynomials - 1, 4 * numCubicPolynomials - 4) = m;


	// via points constraints, C^2 continuous
	Eigen::MatrixXd m1(1, 4);
	Eigen::MatrixXd m2(1, 4);
	Eigen::MatrixXd m3(1, 8);
	Eigen::MatrixXd m4(1, 8);

	m1 << 1, T, pow(T, 2), pow(T, 3);
	m2 << 1, 0, 0, 0;
	m3 << 0, 1, 2 * T, 3 * pow(T, 2), 0, -1, 0, 0;
	m4 << 0, 0, 1, 3 * T, 0, 0, -1, 0;

	if (numCubicPolynomials >= 2){ // 有via points
		for (int i = 1; i< numCubicPolynomials; i++){
			// position constraints
			A.block<1, 4>(4 * i - 2, 4 * i - 4) = m1;
			A.block<1, 4>(4 * i - 2 + 1, 4 * i - 4 + 4) = m2;
			// velocity constraints
			A.block<1, 8>(4 * i - 2 + 2, 4 * i - 4) = m3;
			// acceleration continuous
			A.block<1, 8>(4 * i - 2 + 3, 4 * i - 4) = m4;
		}
	}

	X = Eigen::MatrixXd::Zero(4 * numCubicPolynomials, size); //Ax=b
	X = A.inverse()*B;

	for (int i = 0; i < numCubicPolynomials; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < numCubicPolynomials; i++){
		C[i].resize(4, size);
	}
	for (int i = 1; i <= numCubicPolynomials; i++){
		C[(i - 1)] << X.row((i - 1) * 4), X.row((i - 1) * 4 + 1), X.row((i - 1) * 4 + 2), X.row((i - 1) * 4 + 3);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration, jerk;
	position = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);
	velocity = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);
	jerk = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= numCubicPolynomials; i++){
		for (int j = 0; j <= N; j++){
			position.row(N*(i - 1) + j) = C[(i - 1)].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1)].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1)].row(1)*TimeArray(j, 0) + C[(i - 1)].row(0); // position
			velocity.row(N*(i - 1) + j) = 3 * C[(i - 1)].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1)].row(2)*TimeArray(j, 0) + C[(i - 1)].row(1); // velocity
			acceleration.row(N*(i - 1) + j) = 6 * C[(i - 1)].row(3)*TimeArray(j, 0) + 2 * C[(i - 1)].row(2);  // acceralation
			jerk.row(N*(i - 1) + j) = 6 * C[(i - 1)].row(3);// jerk
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	case 4:
		return jerk;
		break;
	}
}

Eigen::MatrixXd tp::CubicPolynomials(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, double t_total, int select)
{
	double T, t;
	int numCubicPolynomials, N;
	Eigen::MatrixXd A, B, X;
	std::vector<Eigen::MatrixXd> C;
	int size = q0.size();

	numCubicPolynomials = 1;

	T = t_total / numCubicPolynomials; //一小段時間

	B = Eigen::MatrixXd::Zero(4 * numCubicPolynomials, size); //Ax=b

	// 起終點 constraints (position, velocity)
	B.row(0) = q0;
	B.row(1) = q0d;
	B.row(4 * numCubicPolynomials - 2) = qf;
	B.row(4 * numCubicPolynomials - 1) = qfd;

	A = Eigen::MatrixXd::Zero(4 * numCubicPolynomials, 4 * numCubicPolynomials);
	// 起終點 constraints (position, velocity)
	A(0, 0) = 1;
	A(1, 1) = 1;
	Eigen::MatrixXd m(1, 4);
	m << 1, T, pow(T, 2), pow(T, 3);
	A.block<1, 4>(4 * numCubicPolynomials - 2, 4 * numCubicPolynomials - 4) = m;

	m << 0, 1, 2 * T, 3 * pow(T, 2);
	A.block<1, 4>(4 * numCubicPolynomials - 1, 4 * numCubicPolynomials - 4) = m;


	// x = [ a1_0; ~ a1_3; a2_0; ~ a2_3; ... ]

	X = Eigen::MatrixXd::Zero(4 * numCubicPolynomials, size); //Ax=b
	X = A.inverse()*B;

	for (int i = 0; i < numCubicPolynomials; ++i)
		C.push_back(Eigen::MatrixXd());

	for (int i = 0; i < numCubicPolynomials; i++){
		C[i].resize(4, size);
	}
	for (int i = 1; i <= numCubicPolynomials; i++){
		C[(i - 1)] << X.row((i - 1) * 4), X.row((i - 1) * 4 + 1), X.row((i - 1) * 4 + 2), X.row((i - 1) * 4 + 3);
	}

	N = floor(T / 0.005); //N:每一段切的數目

	t = T / N;
	Eigen::MatrixXd position, velocity, acceleration, jerk;
	position = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);
	velocity = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);
	acceleration = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);
	jerk = Eigen::MatrixXd::Zero(N*numCubicPolynomials + 1, size);

	Eigen::MatrixXd TimeArray;
	TimeArray = Eigen::MatrixXd::Zero(N + 1, 1);
	for (int i = 0; i < N; i++){
		TimeArray(i + 1, 0) = TimeArray(i, 0) + t;
	}

	for (int i = 1; i <= numCubicPolynomials; i++){
		for (int j = 0; j <= N; j++){
			position.row(N*(i - 1) + j) = C[(i - 1)].row(3)*pow(TimeArray(j, 0), 3) + C[(i - 1)].row(2)*pow(TimeArray(j, 0), 2) + C[(i - 1)].row(1)*TimeArray(j, 0) + C[(i - 1)].row(0); // position
			velocity.row(N*(i - 1) + j) = 3 * C[(i - 1)].row(3)*pow(TimeArray(j, 0), 2) + 2 * C[(i - 1)].row(2)*TimeArray(j, 0) + C[(i - 1)].row(1); // velocity
			acceleration.row(N*(i - 1) + j) = 6 * C[(i - 1)].row(3)*TimeArray(j, 0) + 2 * C[(i - 1)].row(2);  // acceralation
			jerk.row(N*(i - 1) + j) = 6 * C[(i - 1)].row(3);// jerk
		}
	}

	switch (select)
	{
	case 1:
		return position;
		break;
	case 2:
		return velocity;
		break;
	case 3:
		return acceleration;
		break;
	case 4:
		return jerk;
		break;
	}
}

void tp::interpolation(unsigned number, const std::vector<double>& start, const std::vector<double>& end, std::vector<double>& delx){
	delx.clear();
	for (int i = 0, n = end.size(); i < n; ++i){
		delx.push_back((end[i] - start[i]) / number);
	}
}

void tp::interpolation(int number, const Eigen::VectorXd& qinit, const Eigen::VectorXd& qgoal, std::vector<Eigen::VectorXd>& path){
	path.clear();
	VectorXd delx = (qgoal - qinit) / (number - 1);
	for (int i = 0; i < number; ++i){
		path.push_back(VectorXd(qinit+i*delx));
	}
}
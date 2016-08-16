/*
//���O�W�١GRandomNumGen
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G²��üƲ��;�(for C++98)�A�o�䤣�Q���� �е���C++11 random http://en.cppreference.com/w/cpp/numeric/random
//�ϥΨ禡�w�G�L
*/
#pragma once
#include <stdlib.h>
#include <limits.h> // RAND_MAX
#include <process.h> // _getpid()

class RandomNumGen
{
public:
	RandomNumGen() { srand(_getpid()); }
	RandomNumGen(unsigned seed) { srand(seed); }
	const int operator() (const int range) const{
		return int(range * double(rand()) / RAND_MAX);
	}
	const double operator() (const double min, const double max) const {
		int minInteger = (int)(min * 10000);
		int maxInteger = (int)(max * 10000);
		int randInteger = rand()*rand();
		int diffInteger = maxInteger - minInteger;
		int resultInteger = randInteger % diffInteger + minInteger;
		return resultInteger / 10000.0;
	}
};

extern RandomNumGen rnGen;
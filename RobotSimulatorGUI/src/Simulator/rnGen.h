/*
//類別名稱：RandomNumGen
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：簡單亂數產生器(for C++98)，這邊不想註解 請善用C++11 random http://en.cppreference.com/w/cpp/numeric/random
//使用函式庫：無
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
#include "stdafx.h"
#include "GraspWrenchSpaceQualityMeasure.h"
#include <iostream>
using namespace std;
using namespace orgQhull;

GraspWrenchSpaceQualityMeasure::GraspWrenchSpaceQualityMeasure(){}

GraspWrenchSpaceQualityMeasure::~GraspWrenchSpaceQualityMeasure(){}

bool GraspWrenchSpaceQualityMeasure::compute(const std::vector<std::vector<double> >& vertices){
	if (vertices.size() == 0){
		return false;
	}
	_eps = -DBL_MAX;
	_v = 0.0;
	try{
		_qhull.runQhullXd(vertices, "");
		QhullFacetList facetList = _qhull.facetList();
		for (QhullFacetList::iterator it = facetList.begin(); it != facetList.end(); ++it){
			if (it->getBaseT()->offset > _eps){
				_eps = it->getBaseT()->offset;
			}
		}
		_v = _qhull.volume();
		return true;
	}
	catch(...){
		//cout << "Cannot find convex hull!" << endl;
		return false;
	}
}

double GraspWrenchSpaceQualityMeasure::getEps() const{
	return -_eps;
}

double GraspWrenchSpaceQualityMeasure::getVolume() const{
	return _v;
}
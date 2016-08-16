#include "stdafx.h"
#include "WrenchGenerator.h"
#include <iostream>
#include <set>
using namespace std;
using namespace Eigen;

WrenchGenerator::WrenchGenerator(Object* object, ContactMgr* contactMgr, FrictionCone* frictionCone){
	if (!(object && contactMgr && frictionCone)){
		cout << "Something not init well..." << endl;
	}
	else{
		contactMgr->analysis(object);
		vector<Vector3d> fij, tauij;
		Vector3d objCOM;
		objCOM = object->getCOM3D();
		for (int i = 0, n = contactMgr->size(); i < n; ++i){
			_wrenchs.push_back(vector<VectorXd>());
			fij.clear();
			tauij.clear();
			Vector3d r;
			r << (*contactMgr)[i]->getPoint()[0], (*contactMgr)[i]->getPoint()[1], (*contactMgr)[i]->getPoint()[2];
			r = r - objCOM;
			frictionCone->computeConePoints((*(*contactMgr)[i]), fij);
			for (int j = 0, m = fij.size(); j < m; ++j){
				_wrenchs.back().push_back(VectorXd(6));
				tauij.push_back(Vector3d());
				tauij[j] = r.cross(fij[j]);
				double tauijNorm = tauij[j].norm();
				if (tauijNorm > _tauMax){
					_tauMax = tauijNorm;
				}
				_wrenchs.back()[j] << fij[j][0], fij[j][1], fij[j][2], tauij[j][0], tauij[j][1], tauij[j][2];
			}
		}
		for (auto& i : _wrenchs){
			for (auto& j : i){
				j.tail(3) = j.tail(3) / _tauMax;
			}
		}
	}
}
WrenchGenerator::~WrenchGenerator(){

}

void WrenchGenerator::minkowskiSum(std::vector<std::vector<double> >& wrenchVectorMinkowskiSum){
	vector<VectorXd*> wrenchSum;
	for (int i = 0, n = _wrenchs[0].size(); i < n; ++i){
		wrenchSum.push_back(new VectorXd(_wrenchs[0][i]));
	}

	for (int i = 1, n = _wrenchs.size(); i < n; ++i){
		vector<VectorXd*> wrenchSet;
		for (int j = 0, m = wrenchSum.size(); j < m; ++j){
			for (int k = 0, o = _wrenchs[i].size(); k < o; ++k){
				wrenchSet.push_back(new VectorXd(*wrenchSum[j] + _wrenchs[i][k]));
			}
		}
		for (int i = 0, n = wrenchSum.size(); i < n; ++i){
			delete wrenchSum[i];
		}
		wrenchSum.clear();
		wrenchSum = wrenchSet;
	}

	wrenchVectorMinkowskiSum.clear();
	for (int i = 0, n = wrenchSum.size(); i < n; ++i){
		wrenchVectorMinkowskiSum.push_back(vector<double>());
		for (int j = 0, m = wrenchSum[i]->size(); j < m; ++j){
			wrenchVectorMinkowskiSum[i].push_back((*wrenchSum[i])[j]);
		}
	}

	for (int i = 0, n = wrenchSum.size(); i < n; ++i){
		delete wrenchSum[i];
	}
}

void WrenchGenerator::Union(std::vector<std::vector<double> >& wrenchVector){
	wrenchVector.clear();
	for (int i = 0, n = _wrenchs.size(); i < n; ++i){
		for (int j = 0, m = _wrenchs[i].size(); j < m; ++j){
			wrenchVector.push_back(vector<double>());
			for (int k = 0, o = _wrenchs[i][j].size(); k < o; ++k){
				wrenchVector.back().push_back(_wrenchs[i][j][k]);
			}
		}
	}
}
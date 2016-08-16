#include "stdafx.h"
#include "RRT.h"
#include <stack> //FILO
#include <queue> //FIFO
#include <iostream>
using namespace std;
// global variable
RRTManager* rrtManager = 0;

// RRTManager
RRTManager::RRTManager(){
	rrtManager = this;
}

RRTManager::~RRTManager(){
	rrtManager = 0;
	nodeClear();
}
// Node
RRTManager::Node::Node() : _parent(0) {}

RRTManager::Node::~Node() {}

RRTManager::Node::Node(const Eigen::VectorXd& data) : _parent(0){
	_data = data;
}

void RRTManager::Node::addParent(Node* qparent){
	_parent = qparent;
}

const Eigen::VectorXd& RRTManager::Node::getData() const{
	return _data;
}

Eigen::VectorXd& RRTManager::Node::getData(){
	return _data;
}

RRTManager::Node* RRTManager::Node::getParent(){
	return _parent;
}

// Tree
RRTManager::Tree::Tree(){}
RRTManager::Tree::~Tree(){}
void RRTManager::Tree::init(RRTManager::Node* qinit){
	_vertices.clear();
	_vertices.push_back(qinit);
	rrtManager->_vertices.push_back(qinit);
}

void RRTManager::Tree::addVertex(RRTManager::Node* qnew){
	_vertices.push_back(qnew);
	rrtManager->_vertices.push_back(qnew);
}

void RRTManager::Tree::addEdge(RRTManager::Node* qnear, RRTManager::Node* qnew){
	qnew->addParent(qnear);
	rrtManager->_edges.insert(NodePtrPair(qnear, qnew));
}

std::vector<RRTManager::Node*>& RRTManager::Tree::getVertices(){
	return _vertices;
}

RRTManager::Node* RRTManager::Tree::nearestNeighbor(const RRTManager::Node& q){
	double minDis = DBL_MAX, dis = 0.0;
	size_t index = 0;
	for (size_t i = 0, n = _vertices.size(); i < n; ++i){
		dis = (q.getData() - _vertices[i]->getData()).norm();
		if (dis < minDis){
			minDis = dis;
			index = i;
		}
	}
	return _vertices[index];
}

RRTManager::State RRTManager::Tree::extend(const RRTManager::Node& q){
	RRTManager::Node *qnear = nearestNeighbor(q), *qnew = 0;
	if (rrtManager->newConfig(q, qnear, qnew)){
		addVertex(qnew);
		addEdge(qnear, qnew);
		if (qnew->getData() == q.getData()){
			rrtManager->_tbViaNode = qnew;
			return RRTManager::REACHED;
		}
		else
			return RRTManager::ADVANCED;
	}
	return RRTManager::TRAPPED;
}

RRTManager::State RRTManager::Tree::extend(const RRTManager::Node& q, RRTManager::Node& Qnew, RRTManager::Node*& qnew){
	Node *qnear = nearestNeighbor(q);
	if (rrtManager->newConfig(q, qnear, qnew)){
		Qnew = *qnew;
		addVertex(qnew);
		addEdge(qnear, qnew);
		if (qnew->getData() == q.getData())
			return REACHED;
		else
			return ADVANCED;
	}
	return TRAPPED;
}

// manager
void RRTManager::buildRRT(RRTManager::Tree& tree, const RRTManager::Node& qinit){
	nodeClear();
	Node* qnewInit = new Node(qinit);
	tree.init(qnewInit);
	for (int i = 0; i < _K; ++i){
		Node qrand;
		randomConfig(qrand);
		tree.extend(qrand);
	}
}

bool RRTManager::RRTConnectPlanner(RRTManager::Tree* Ta, RRTManager::Tree* Tb, const RRTManager::Node& qinit, const RRTManager::Node& qgoal, std::vector<Eigen::VectorXd>& path){
	nodeClear();
	Node *Qinit = new Node(qinit), *Qgoal = new Node(qgoal);
	Ta->init(Qinit);
	Tb->init(Qgoal);
	Node qrand, Qnew;
	Node* taViaNode;
	for (int i = 0; i < _K; ++i){
		randomConfig(qrand);
		if (Ta->extend(qrand, Qnew, taViaNode) != TRAPPED){
			if (connect(*Tb, Qnew) == REACHED){
				if (Ta->getVertices()[0] == Qgoal){
					swap(taViaNode, _tbViaNode);
				}
				Node* tempNode;
				stack<Node*> uperHalfPath;
				tempNode = taViaNode;
				while (tempNode->getParent()){
					uperHalfPath.push(tempNode);
					tempNode = tempNode->getParent();
				}
				uperHalfPath.push(tempNode);
				queue<Node*> lowerHalfPath;
				tempNode = _tbViaNode->getParent();
				while (tempNode->getParent()){
					lowerHalfPath.push(tempNode);
					tempNode = tempNode->getParent();
				}
				lowerHalfPath.push(tempNode);
				while (!uperHalfPath.empty()){
					path.push_back(uperHalfPath.top()->getData());
					uperHalfPath.pop();
				}
				while (!lowerHalfPath.empty()){
					path.push_back(lowerHalfPath.front()->getData());
					lowerHalfPath.pop();
				}
				//cout << "iter:" << i << endl;
				return true;
			}
		}
		swap(Ta, Tb);
	}
	return false;
}

void RRTManager::setDim(int dim){
	_dim = dim;
}

void RRTManager::setEps(double eps){
	_eps = eps;
}

void RRTManager::setIterMax(int K){
	_K = K;
}

RRTManager::State RRTManager::connect(RRTManager::Tree& T, const RRTManager::Node& q){
	State S;
	do{
		S = T.extend(q);
	} while (S == ADVANCED);
	return S;
}

void RRTManager::swap(RRTManager::Tree*& Ta, RRTManager::Tree*& Tb){
	Tree* temp = Ta;
	Ta = Tb;
	Tb = temp;
}

void RRTManager::swap(RRTManager::Node*& nodea, RRTManager::Node*& nodeb){
	Node* temp = nodea;
	nodea = nodeb;
	nodeb = temp;
}

bool RRTManager::newConfig(const RRTManager::Node& q, RRTManager::Node* qnear, RRTManager::Node*& qnew){
	Eigen::VectorXd minus = q.getData() - qnear->getData();
	Node Qnew;
	if (minus.norm() <= _eps){
		Qnew = Node(q);
	}
	else{
		minus.normalize();
		Qnew = Node(qnear->getData() + _eps*minus);
	}
	if (!checkCollision(Qnew)){
		qnew = new Node(Qnew);
		return true;
	}
	else
		return false;
}

void RRTManager::nodeClear(){
	for (int i = 0, n = _vertices.size(); i < n; ++i){
		delete _vertices[i];
	}
	_vertices.clear();
	_edges.clear();
	_tbViaNode = 0;
}

void RRTManager::randomConfig(RRTManager::Node& qrand){
	qrand.getData() = Eigen::VectorXd::Random(_dim) * 300; // ¼È®É
}

bool RRTManager::checkCollision(const RRTManager::Node& q){
	if (q.getData()[0] >= 0 && q.getData()[0] <= 150){
		if (q.getData()[1] >= -60 && q.getData()[1] <= 120){
			return true;
		}
	}
	return false;
}


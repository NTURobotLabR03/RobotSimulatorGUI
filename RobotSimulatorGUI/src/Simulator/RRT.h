/*
//類別名稱：RRTManager
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：管理所有RRT Tree Node的類別
//      裡面有基本RRT algorithm, 以及RRT-Connect Algorithm
//使用函式庫：Eigen
*/
#pragma once
#include <Eigen\Dense>
#include <vector>
#include <map>

class RRTManager{
	friend class Node;
	friend class Tree;
public:
	RRTManager();
	~RRTManager();

	enum State{
		REACHED,
		ADVANCED,
		TRAPPED,
	};

	class Node{
		friend class Tree;
	public:
		Node();
		Node(const Eigen::VectorXd& data);
		~Node();
		void addParent(Node* qparent);
		Node* getParent();
		const Eigen::VectorXd& getData() const;
		Eigen::VectorXd& getData();
	private:
		Eigen::VectorXd _data;
		Node* _parent;
	};

	typedef std::pair<Node*, Node*> NodePtrPair;

	class Tree{
	public:
		Tree();
		~Tree();
		void init(Node* qinit);
		void addVertex(Node* qnew);
		static void addEdge(Node* qnear, Node* qnew);
		std::vector<Node*>& getVertices();
		Node* nearestNeighbor(const Node& q);
		State extend(const Node& q);
		State extend(const Node& q, Node& Qnew, Node*& qnew);
	private:
		std::vector<Node*> _vertices;
	};
	
	void buildRRT(Tree& tree, const Node& qinit);
	virtual bool RRTConnectPlanner(Tree* Ta, Tree* Tb, const Node& qinit, const Node& qgoal, std::vector<Eigen::VectorXd>& path);
	void setDim(int dim);
	void setEps(double eps);
	void setIterMax(int K);

protected:
	State connect(Tree& T, const Node& q);
	void swap(Tree*& Ta, Tree*& Tb);
	void swap(Node*& nodea, Node*& nodeb);
	bool newConfig(const Node& q, Node* qnear, Node*& qnew);
	void nodeClear();
	virtual void randomConfig(Node& qrand);
	virtual bool checkCollision(const Node& q);

	int _dim;
	double _eps;
	int _K; // iterMax
	std::vector<Node*> _vertices;
	std::multimap<Node*, Node*> _edges;
	// rrt connect using 
	Node* _tbViaNode;
};

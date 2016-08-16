/*
//類別名稱：Robot
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：建立描述Robot的Class
//使用函式庫：Eigen, Bullet
*/
#pragma once
#include "frame.h"
#include <Eigen/StdVector>

namespace Eigen{
	typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Matrix4dList;
	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dList;
}
namespace rbt{
	class Robot
	{
	public:
		// 建構會先push_back一個BasicFrame在_allFrameList跟_baseFrameList
		Robot();
		Robot(const Eigen::Matrix4d& T);
		~Robot();
		// 增加普通不能動的Frame
		void addBasicFrame(const Eigen::Matrix4d& T, int parentId = -1);
		// 增加一個對動的軸
		void addRevoluteFrame(double a, double alpha, double d, double theta, double min, double max, int parentId = -1);
		// 增加一個被動的軸
		void addPassiveFrame(double a, double alpha, double d, double theta, double ratio, int activeParentId, int parentId = -1);
		// DH建好要call一次這個function
		void searchEE();
		// 畫棒棒用(openGL) 現在已經停用的function
		void drawRobot() const;
		// 回傳自由度數量
		unsigned DOFsize() const;
		// 回傳End effector數量
		unsigned EEsize() const;
		// operator [] 會回傳第i個Frame
		const DHFrame& operator[] (unsigned i) const;
		DHFrame& operator[] (unsigned i);
		// 計算End effectors的位置
		void getFK(std::vector<Eigen::Vector3d>& endEffectorPos) const;
		// 計算End effectors的姿態(position + orientation)
		void getFK(Eigen::Matrix4dList& endEffectorMat) const;
		// 得到Jacobian Matrix(只有position的部分)
		void getJacobian(Eigen::MatrixXd& J) const;
		// 得到Jacobian Matrix
		void getJacobianWO(Eigen::MatrixXd& J) const;
		// reset robot
		virtual void reset();
		// 解IK (position) 將final configuration存入 jointValue
		void dlsIK(const std::vector<Eigen::Vector3d>& targetPos, std::vector<double>& jointValue);
		// 解IK 將final configuration存入 jointValue
		void dlsIK(const Eigen::Matrix4dList& targetPosMat, std::vector<double>& jointValue);

		// 20160421
		std::vector<Frame*>& getAllFrameList();
		// 每當有frame轉動, 一定要call這個function 會幫忙更新Object的位置
		virtual void setObjectPose();
	protected:
		std::vector<Frame*> _allFrameList, _baseFrameList;
		std::vector<DHFrame*> _jointFrameList;
		std::vector<PDHFrame*> _passiveFrameList;
		std::vector<EndEffector*> _endEffectorList;
	};

	void transMat2Vec(const Eigen::Matrix4d& transMat, Eigen::VectorXd& P);
}
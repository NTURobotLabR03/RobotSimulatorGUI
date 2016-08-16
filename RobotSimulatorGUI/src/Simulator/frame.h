/*
//類別名稱：Frame, BasicFrame(繼承Frame), DHFrame(繼承Frame), PDHFrame(繼承Frame), EndEffector
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：建立各種坐標系
//      並存入上一個坐標系看現在坐標系的Transformation Matrix
//      Object類別的引進可以存入對應的CAD圖
//使用函式庫：Eigen, Bullet(for Object類別)
*/
#pragma once
#include "Eigen/Dense"
#include "Object.h"
#include <GL\freeglut.h>
#include <math.h>
#include <vector>

//只是個namespace
namespace rbt{
	//============事先宣告=============
	class Frame;
	class BasicFrame;
	class PDHFrame;
	class DHFrame;
	class EndEffector;
	//=================================

	class Frame
	{
	public:
		//如果class裡面有Eigen fix vector or array or matrix就要使用這個巨集
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // for eigen fix matrix
		Frame();
		virtual ~Frame();
		// 畫棒棒用(openGL) 現在已經停用的function
		virtual void draw() const = 0;
		// 增加Node與下個Node關係
		void addChild(Frame* child);
		// traversal 用
		void getFK(std::vector<Eigen::Vector3d>& endEffectorPos, const Eigen::Matrix4d& T) const;
		void getFK(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& endEffectorMat, const Eigen::Matrix4d& T) const;
		void searchEE(std::vector<Frame*> path, std::vector<EndEffector*>& list);
		// 回傳上一個frame看此frame的transformation matrix
		const Eigen::Matrix4d& getTransMat();
		// 回傳是否是可控的軸
		virtual bool isDrive();
		// 回傳是否是被動的軸
		virtual bool isPassive();

		// 20160421
		// 設定這個frame對應的Object(用於碰撞偵測)
		void setObject(Object* obj);
		// 回傳對應的Object
		Object* getObject();
		// 當_transMat更動時 一定要call這個function 會幫忙更新Object的位置
		virtual void setObjectPose(Eigen::Matrix4d& T) = 0;

	protected:
		Eigen::Matrix4d _transMat;
		std::vector<Frame*> _children;
		Object* _object = 0;
	};

	class BasicFrame :public Frame
	{
	public:
		BasicFrame();
		BasicFrame(const Eigen::Matrix4d& T);
		~BasicFrame();
		// 畫棒棒用(openGL) 現在已經停用的function
		void draw() const;

		//20160421
		// 當_transMat更動時 一定要call這個function 會幫忙更新Object的位置
		void setObjectPose(Eigen::Matrix4d& T);
		// 20160422
		// 改變_transMat的值
		void setTransMat(const Eigen::Matrix4d& transMat);
	};


	class PDHFrame :public Frame // Passive DH Frame
	{
	public:
		PDHFrame();
		PDHFrame(double a, double alpha, double d, double theta, double ratio, DHFrame* activeParent);
		~PDHFrame();
		// 畫棒棒用(openGL) 現在已經停用的function
		void draw() const;
		// 回傳主動軸的id
		unsigned getDriveId();
		// 回傳與主動軸的轉動比例
		double getRatio();
		// 會回傳true, 因為是被動軸
		bool isPassive();
		// 會因應主動軸改變角度而更改_transMat
		void updateTransMat();
		//20160421
		// 當_transMat更動時 一定要call這個function 會幫忙更新Object的位置
		void setObjectPose(Eigen::Matrix4d& T);
		//20160423
		// 回傳主動軸的指標
		DHFrame* getActiveParent();
	private:
		double _a, _alpha, _d, _theta;
		double _ratio;
		double* _parentCmd;
		DHFrame* _activeParent;
	};

	class DHFrame :public Frame
	{
	public:
		DHFrame();
		DHFrame(double a, double alpha, double d, double theta, double min, double max);
		~DHFrame();
		// 設定現在轉的角度 th = cmd
		void setCmd(double cmd);
		// 設定轉的角度 th += cmd
		void updateCmd(double cmd);
		// 得到現在轉多少度
		double getCmd();
		// 設定此軸的id
		void setId(unsigned id);
		// 回傳id
		unsigned getId();
		// 畫棒棒用(openGL) 現在已經停用的function
		void draw() const;
		// 會回傳true, 因為是主動軸
		bool isDrive();
		// 給予_cmd的指標 用於給PDHFrame用
		double* getCmdAddress();
		// 增加被動軸的連結
		void addPassiveChild(PDHFrame* child);
		// 設定joint limits
		void getCmdRange(double& min, double& max);
		//20160421
		// 當_transMat更動時 一定要call這個function 會幫忙更新Object的位置
		void setObjectPose(Eigen::Matrix4d& T);
		// 回傳存放被動軸的容器
		std::vector<PDHFrame*>& getPassiveChildren();
	private:
		double _a, _alpha, _d, _theta;
		double _max, _min, _cmd;
		unsigned _id;
		std::vector<PDHFrame*> _passiveChildren;
		void updateTransMat();
	};

	// 此class幫忙找尋end effectors中間所經過的frame
	class EndEffector
	{
	public:
		EndEffector();
		EndEffector(const std::vector<Frame*>& p);
		~EndEffector();
	
		void addPath(const std::vector<Frame*>& p);
		const std::vector<Frame*>& getPath();
	private:
		std::vector<Frame*> _path;
	};

	// 以下function都沒用到
	void Draw_Cute_Axis(float LINK_LENGTH);  //畫軸 實驗室創
	void drawCylinder(double Radius, double Height, float red = 1, float green = 0, float blue = 0);
	void drawRect(double Length, double Width, double Height, float red = 1, float green = 0, float blue = 0);
}
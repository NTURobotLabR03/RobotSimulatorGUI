/*
//類別名稱：UISimulator(繼承TrajectoryPlanningSimulator)
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：與MFC結合，新增許多溝通的Function，也是最後的Simulator
//使用函式庫：FreeGLUT, Bullet, Eigen, Qhull
*/
#pragma once
#include "TrajectoryPlanningSimulator.h"
#include "HandArmState.h"
#include "PathPlayBack.h"

class UISimulator : public TrajectoryPlanningSimulator{
public:
	//如果class裡面有Eigen fix vector or array or matrix就要使用這個巨集
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	UISimulator();
	virtual ~UISimulator();

	// FreeGlut Callbacks
	// 鍵盤事件
	virtual void Keyboard(unsigned char key, int x, int y) override;
	// Special Key鍵盤事件
	virtual void Special(int key, int x, int y) override;
	// 平常沒事的callback function
	virtual void Idle() override;

	// robot env function
	// 初始化robot 跟 桌子
	virtual void InitRobotEnv() override;

	// ui bottom interface function
	// 將某個Object回復碰撞偵測
	void restoreObject(Object* obj);
	// 將某個Object暫時失去碰撞偵測效果
	void tempDeleteObject(Object* obj);
	// 回放某個路徑過程
	void runPath(PathPlayBack* path);
	// 獲得規劃好路徑
	const std::vector<Eigen::VectorXd>& getPath() const;
	// 解Arm IK, command是下在_aidFrame上
	void solveIK();
	// 獲得Arm Endeffector的位置
	void getArmFK(Eigen::Matrix4d& T);
	// 獲得輔助Frame的位置
	Eigen::Matrix4d& getAidFrame();
	// 改變Arm的Joint Values
	void setArmJoint(const std::vector<double>& values);
	// 改變Arm的Joint Values
	void setArmJoint(const Eigen::VectorXd& values);
	// 得到Arm現在的Joint Values
	void getArmJoint(std::vector<double>& values);
	// 得到Arm現在的Joint Values
	void getArmJoint(Eigen::VectorXd& values);
	// 改變Hand現在的Joint Values
	void setHandJoint(const std::vector<double>& values);
	// 改變Hand現在的Joint Values
	void setHandJoint(const Eigen::VectorXd& values);
	// 得到Hand現在的Joint Values
	void getHandJoint(std::vector<double>& values);
	// 得到Hand現在的Joint Values
	void getHandJoint(Eigen::VectorXd& values);
	// 得到目標Object的指標
	Object* getTarget();
	// 設定目標Object
	void setTarget(Object* object);
	// 做Path Pruning, command是下在 _path上
	void pathSmoothing();
	// 做grasp planning, command 目標下在 _target上
	double graspPlanning();
	// 做rrt-connect 目標路徑存在 _path裡
	void pathPlanning(const HandArmState& Qinit, const HandArmState& Qgoal);
	// hand抓下去
	void handGrasp();
	// hand鬆開
	void handRelease();
	// 畫出path的經過點 用紅色球球畫
	void setPathFlag(bool flag);
	// 刪除某物體 (解除碰撞偵測以及openGL的圖)
	void deleteObject(Object* obj);
	// 把RRT的Node都畫出來 會有bug所以先按在Path Planning會比較好
	void setRRTDrawFlag(bool flag);
	// 把contact的點畫出來
	void setContactMgrDrawFlag(bool flag);
	// 畫出所有物體的AABB
	void setDrawAabbFlag();
	// 劃出_aidFrame的位置 用來解IK用
	void setAidFlag(bool flag);
	// 回傳桌子的位置
	Object* getTable();
protected:
	// 用在IDLE function裡
	void allDebugDraw();
	// 同上
	void aidFrameDraw();
	// draw motion function
	bool _pathFlag = false, _aidFlag = false;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> _pathFK;
	Eigen::Matrix4d _aidFrame = Eigen::Matrix4d::Identity();
};
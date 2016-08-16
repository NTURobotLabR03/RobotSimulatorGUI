/*
//���O�W�١GUISimulator(�~��TrajectoryPlanningSimulator)
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�PMFC���X�A�s�W�\�h���q��Function�A�]�O�̫᪺Simulator
//�ϥΨ禡�w�GFreeGLUT, Bullet, Eigen, Qhull
*/
#pragma once
#include "TrajectoryPlanningSimulator.h"
#include "HandArmState.h"
#include "PathPlayBack.h"

class UISimulator : public TrajectoryPlanningSimulator{
public:
	//�p�Gclass�̭���Eigen fix vector or array or matrix�N�n�ϥγo�ӥ���
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	UISimulator();
	virtual ~UISimulator();

	// FreeGlut Callbacks
	// ��L�ƥ�
	virtual void Keyboard(unsigned char key, int x, int y) override;
	// Special Key��L�ƥ�
	virtual void Special(int key, int x, int y) override;
	// ���`�S�ƪ�callback function
	virtual void Idle() override;

	// robot env function
	// ��l��robot �� ��l
	virtual void InitRobotEnv() override;

	// ui bottom interface function
	// �N�Y��Object�^�_�I������
	void restoreObject(Object* obj);
	// �N�Y��Object�Ȯɥ��h�I�������ĪG
	void tempDeleteObject(Object* obj);
	// �^��Y�Ӹ��|�L�{
	void runPath(PathPlayBack* path);
	// ��o�W���n���|
	const std::vector<Eigen::VectorXd>& getPath() const;
	// ��Arm IK, command�O�U�b_aidFrame�W
	void solveIK();
	// ��oArm Endeffector����m
	void getArmFK(Eigen::Matrix4d& T);
	// ��o���UFrame����m
	Eigen::Matrix4d& getAidFrame();
	// ����Arm��Joint Values
	void setArmJoint(const std::vector<double>& values);
	// ����Arm��Joint Values
	void setArmJoint(const Eigen::VectorXd& values);
	// �o��Arm�{�b��Joint Values
	void getArmJoint(std::vector<double>& values);
	// �o��Arm�{�b��Joint Values
	void getArmJoint(Eigen::VectorXd& values);
	// ����Hand�{�b��Joint Values
	void setHandJoint(const std::vector<double>& values);
	// ����Hand�{�b��Joint Values
	void setHandJoint(const Eigen::VectorXd& values);
	// �o��Hand�{�b��Joint Values
	void getHandJoint(std::vector<double>& values);
	// �o��Hand�{�b��Joint Values
	void getHandJoint(Eigen::VectorXd& values);
	// �o��ؼ�Object������
	Object* getTarget();
	// �]�w�ؼ�Object
	void setTarget(Object* object);
	// ��Path Pruning, command�O�U�b _path�W
	void pathSmoothing();
	// ��grasp planning, command �ؼФU�b _target�W
	double graspPlanning();
	// ��rrt-connect �ؼи��|�s�b _path��
	void pathPlanning(const HandArmState& Qinit, const HandArmState& Qgoal);
	// hand��U�h
	void handGrasp();
	// hand�P�}
	void handRelease();
	// �e�Xpath���g�L�I �ά���y�y�e
	void setPathFlag(bool flag);
	// �R���Y���� (�Ѱ��I�������H��openGL����)
	void deleteObject(Object* obj);
	// ��RRT��Node���e�X�� �|��bug�ҥH�����bPath Planning�|����n
	void setRRTDrawFlag(bool flag);
	// ��contact���I�e�X��
	void setContactMgrDrawFlag(bool flag);
	// �e�X�Ҧ����骺AABB
	void setDrawAabbFlag();
	// ���X_aidFrame����m �ΨӸ�IK��
	void setAidFlag(bool flag);
	// �^�Ǯ�l����m
	Object* getTable();
protected:
	// �ΦbIDLE function��
	void allDebugDraw();
	// �P�W
	void aidFrameDraw();
	// draw motion function
	bool _pathFlag = false, _aidFlag = false;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> _pathFK;
	Eigen::Matrix4d _aidFrame = Eigen::Matrix4d::Identity();
};
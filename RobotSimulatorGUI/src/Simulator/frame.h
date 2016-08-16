/*
//���O�W�١GFrame, BasicFrame(�~��Frame), DHFrame(�~��Frame), PDHFrame(�~��Frame), EndEffector
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��G�إߦU�ا��Шt
//      �æs�J�W�@�ӧ��Шt�ݲ{�b���Шt��Transformation Matrix
//      Object���O���޶i�i�H�s�J������CAD��
//�ϥΨ禡�w�GEigen, Bullet(for Object���O)
*/
#pragma once
#include "Eigen/Dense"
#include "Object.h"
#include <GL\freeglut.h>
#include <math.h>
#include <vector>

//�u�O��namespace
namespace rbt{
	//============�ƥ��ŧi=============
	class Frame;
	class BasicFrame;
	class PDHFrame;
	class DHFrame;
	class EndEffector;
	//=================================

	class Frame
	{
	public:
		//�p�Gclass�̭���Eigen fix vector or array or matrix�N�n�ϥγo�ӥ���
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // for eigen fix matrix
		Frame();
		virtual ~Frame();
		// �e�δΥ�(openGL) �{�b�w�g���Ϊ�function
		virtual void draw() const = 0;
		// �W�[Node�P�U��Node���Y
		void addChild(Frame* child);
		// traversal ��
		void getFK(std::vector<Eigen::Vector3d>& endEffectorPos, const Eigen::Matrix4d& T) const;
		void getFK(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& endEffectorMat, const Eigen::Matrix4d& T) const;
		void searchEE(std::vector<Frame*> path, std::vector<EndEffector*>& list);
		// �^�ǤW�@��frame�ݦ�frame��transformation matrix
		const Eigen::Matrix4d& getTransMat();
		// �^�ǬO�_�O�i�����b
		virtual bool isDrive();
		// �^�ǬO�_�O�Q�ʪ��b
		virtual bool isPassive();

		// 20160421
		// �]�w�o��frame������Object(�Ω�I������)
		void setObject(Object* obj);
		// �^�ǹ�����Object
		Object* getObject();
		// ��_transMat��ʮ� �@�w�ncall�o��function �|������sObject����m
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
		// �e�δΥ�(openGL) �{�b�w�g���Ϊ�function
		void draw() const;

		//20160421
		// ��_transMat��ʮ� �@�w�ncall�o��function �|������sObject����m
		void setObjectPose(Eigen::Matrix4d& T);
		// 20160422
		// ����_transMat����
		void setTransMat(const Eigen::Matrix4d& transMat);
	};


	class PDHFrame :public Frame // Passive DH Frame
	{
	public:
		PDHFrame();
		PDHFrame(double a, double alpha, double d, double theta, double ratio, DHFrame* activeParent);
		~PDHFrame();
		// �e�δΥ�(openGL) �{�b�w�g���Ϊ�function
		void draw() const;
		// �^�ǥD�ʶb��id
		unsigned getDriveId();
		// �^�ǻP�D�ʶb����ʤ��
		double getRatio();
		// �|�^��true, �]���O�Q�ʶb
		bool isPassive();
		// �|�]���D�ʶb���ܨ��צӧ��_transMat
		void updateTransMat();
		//20160421
		// ��_transMat��ʮ� �@�w�ncall�o��function �|������sObject����m
		void setObjectPose(Eigen::Matrix4d& T);
		//20160423
		// �^�ǥD�ʶb������
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
		// �]�w�{�b�઺���� th = cmd
		void setCmd(double cmd);
		// �]�w�઺���� th += cmd
		void updateCmd(double cmd);
		// �o��{�b��h�֫�
		double getCmd();
		// �]�w���b��id
		void setId(unsigned id);
		// �^��id
		unsigned getId();
		// �e�δΥ�(openGL) �{�b�w�g���Ϊ�function
		void draw() const;
		// �|�^��true, �]���O�D�ʶb
		bool isDrive();
		// ����_cmd������ �Ω�PDHFrame��
		double* getCmdAddress();
		// �W�[�Q�ʶb���s��
		void addPassiveChild(PDHFrame* child);
		// �]�wjoint limits
		void getCmdRange(double& min, double& max);
		//20160421
		// ��_transMat��ʮ� �@�w�ncall�o��function �|������sObject����m
		void setObjectPose(Eigen::Matrix4d& T);
		// �^�Ǧs��Q�ʶb���e��
		std::vector<PDHFrame*>& getPassiveChildren();
	private:
		double _a, _alpha, _d, _theta;
		double _max, _min, _cmd;
		unsigned _id;
		std::vector<PDHFrame*> _passiveChildren;
		void updateTransMat();
	};

	// ��class������Mend effectors�����Ҹg�L��frame
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

	// �H�Ufunction���S�Ψ�
	void Draw_Cute_Axis(float LINK_LENGTH);  //�e�b ����ǳ�
	void drawCylinder(double Radius, double Height, float red = 1, float green = 0, float blue = 0);
	void drawRect(double Length, double Width, double Height, float red = 1, float green = 0, float blue = 0);
}
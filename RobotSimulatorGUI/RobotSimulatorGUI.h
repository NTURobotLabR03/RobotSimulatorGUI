
// RobotSimulatorGUI.h : PROJECT_NAME ���ε{�����D�n���Y��
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�� PCH �]�t���ɮ׫e���]�t 'stdafx.h'"
#endif

#include "resource.h"		// �D�n�Ÿ�


// CRobotSimulatorGUIApp: 
// �аѾ\��@�����O�� RobotSimulatorGUI.cpp
//

class CRobotSimulatorGUIApp : public CWinApp
{
public:
	CRobotSimulatorGUIApp();

// �мg
public:
	virtual BOOL InitInstance();

// �{���X��@

	DECLARE_MESSAGE_MAP()
};

extern CRobotSimulatorGUIApp theApp;
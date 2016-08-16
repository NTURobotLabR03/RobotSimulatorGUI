/*
//類別名稱：無
//作者：Chris Dickinson
//日期：2016/08/15
//目的：由於FreeGLUT的Callback Function都必須為靜態函式
//      這邊用幾個簡單的靜態函式跟靜態物件 成功呼叫local物件的callback function
//      glut的main在這邊執行
//使用函式庫：FreeGLUT
*/
#pragma once
#include "Simulator.h"
#include "ThreadControl.h"
// 全域變數
// global pointer to our application object
static Simulator* g_pApp;

/** Various static functions that will be handed to FreeGLUT to be called
during various events (our callbacks). Each calls an equivalent function
in our (global) application object. **/
static void KeyboardCallback(unsigned char key, int x, int y) {
	g_pApp->Keyboard(key, x, y);
}
static void KeyboardUpCallback(unsigned char key, int x, int y) {
	g_pApp->KeyboardUp(key, x, y);
}
static void SpecialCallback(int key, int x, int y) {
	g_pApp->Special(key, x, y);
}
static void SpecialUpCallback(int key, int x, int y) {
	g_pApp->SpecialUp(key, x, y);
}
static void ReshapeCallback(int w, int h) {
	g_pApp->Reshape(w, h);
}
static void IdleCallback() {
	g_pApp->Idle();
}
static void MouseCallback(int button, int state, int x, int y) {
	g_pApp->Mouse(button, state, x, y);
}
static void MotionCallback(int x, int y) {
	g_pApp->Motion(x, y);
}
static void DisplayCallback(void) {
	g_pApp->Display();
}

// our custom-built 'main' function, which accepts a reference to a 
// BulletOpenGLApplication object.
void glutmain(int argc, char **argv, int width, int height, const char* title, Simulator* pApp) {
	// store the application object so we can
	// access it globally
	gMutex.lock();
	g_pApp = pApp;

	// initialize the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	// perform custom initialization our of application
	g_pApp->Initialize();
	gMutex.unlock();
	// give our static
	glutKeyboardFunc(KeyboardCallback);
	glutKeyboardUpFunc(KeyboardUpCallback);
	glutSpecialFunc(SpecialCallback);
	glutSpecialUpFunc(SpecialUpCallback);
	glutReshapeFunc(ReshapeCallback);
	glutIdleFunc(IdleCallback);
	glutMouseFunc(MouseCallback);
	glutPassiveMotionFunc(MotionCallback);
	glutMotionFunc(MotionCallback);
	glutDisplayFunc(DisplayCallback);

	// perform one render before we launch the application
	g_pApp->Idle();

	// hand application control over to the FreeGLUT library.
	// This function remains in a while-loop until the
	// application is exited.
	glutMainLoop();
	return;
}
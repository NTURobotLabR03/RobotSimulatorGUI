/*
//類別名稱：無
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：FreeGLUT Library的所有include檔
//      以及一些輔助繪圖的function
//使用函式庫：FreeGLUT
*/
#pragma once
#include <Windows.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#define GLUT_TOT_BUTTON 3

// 這邊放輔助函式
// 畫箭頭
void Arrow(GLdouble x1, GLdouble y1, GLdouble z1, GLdouble x2, GLdouble y2, GLdouble z2, GLdouble D);
// 畫軸
void drawAxes(GLdouble length);
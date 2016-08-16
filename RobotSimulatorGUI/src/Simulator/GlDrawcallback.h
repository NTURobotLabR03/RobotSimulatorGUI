/*
//類別名稱：GlDrawcallback(繼承btTriangleCallbackBullet)
//作者：Chris Dickinson
//日期：2016/08/15
//目的：如果是匯CAD圖進來的btCollisionShape
//      會經過這個類別與OpenGL溝通，畫出mesh的各個三角形
//使用函式庫：Bullet, FreeGLUT
*/
#pragma once
#include "BulletHeader.h"
#include "FreeGlutHeader.h"
#include <iostream>
using namespace std;

class GlDrawcallback : public btTriangleCallback
{

public:

	bool	m_wireframe;

	GlDrawcallback()
		:m_wireframe(false)
	{
	}

	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{

		(void)triangleIndex;
		(void)partId;


		if (m_wireframe)
		{
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 1, 0);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 0, 1);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		}
		else
		{
			glBegin(GL_TRIANGLES);
			//glColor3f(1, 1, 1);

			btVector3 normal = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);
			if (!normal.fuzzyZero()){
				normal.normalize();
				glNormal3f(normal.getX(), normal.getY(), normal.getZ());
			}
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());

			//glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			//glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			//glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		}
	}
};
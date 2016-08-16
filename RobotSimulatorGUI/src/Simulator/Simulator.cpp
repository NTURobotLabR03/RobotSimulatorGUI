#include "stdafx.h"
#include "Simulator.h"
#include <iostream>
using namespace std;
using namespace Eigen;

Simulator::Simulator(){}

Simulator::~Simulator(){
	// shutdown the physics system
	ShutdownBullet();
	delete _arm;
	delete _hand;
}

void Simulator::Keyboard(unsigned char key, int x, int y){
	// bullet dubug draw
	// toggle wireframe debug drawing
	//m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawWireframe);
	// toggle AABB debug drawing
	//m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawAabb);

	// This function is called by FreeGLUT whenever
	// generic keys are pressed down.
	switch (key) {
	case 'q':
		(*_arm)[0].updateCmd(0.01);
		break;
	case 'a':
		(*_arm)[0].updateCmd(-0.01);
		break;
	case 'w':
		(*_arm)[1].updateCmd(0.01);
		break;
	case 's'://b
		(*_arm)[1].updateCmd(-0.01);
		break;
	case 'e':
		(*_arm)[2].updateCmd(0.01);
		break;
	case 'd':
		(*_arm)[2].updateCmd(-0.01);
		break;
	case 'r':
		(*_arm)[3].updateCmd(0.01);
		break;
	case 'f':
		(*_arm)[3].updateCmd(-0.01);
		break;
	case 't':
		(*_arm)[4].updateCmd(0.01);
		break;
	case 'g':
		(*_arm)[4].updateCmd(-0.01);
		break;
	case 'y':
		(*_arm)[5].updateCmd(0.01);
		break;
	case 'h':
		(*_arm)[5].updateCmd(-0.01);
		break;
	case char(13) : // enter key
		_hand->reset();
		_hand->setMotionFlag(true);
		break;
	}
	_arm->setObjectPose();
	_hand->setObjectPose();
	_hand->connectToArm(_arm);
}

void Simulator::KeyboardUp(unsigned char key, int x, int y){

}

void Simulator::Special(int key, int x, int y){

}

void Simulator::SpecialUp(int key, int x, int y){

}

void Simulator::Reshape(int w, int h){
	// this function is called once during application intialization
	// and again every time we resize the window

	// grab the screen width/height
	m_screenWidth = w;
	m_screenHeight = h;
	// set the viewport
	glViewport(0, 0, w, h);
	// update the camera
	UpdateCamera();
}

void Simulator::Idle(){
	// this function is called frequently, whenever FreeGlut
	// isn't busy processing its own events. It should be used
	// to perform any updating and rendering tasks

	// clear the backbuffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_pWorld){
		// check for any new collisions/separations
		_hand->grasp();
		CheckForCollisionEvents();
	}

	// update the camera
	UpdateCamera();

	// render the scene
	RenderScene();

	// swap the front and back buffers
	glutSwapBuffers();
}

void Simulator::RenderScene(){
	// create an array of 16 floats (representing a 4x4 matrix)
	btScalar transform[16];

	for (const auto& it : m_objects){
		it->GetTransform(transform);

		// get data from the object and draw it
		DrawShape(transform, it->getShape(), it->getColor());
	}

	// after rendering all game objects, perform debug rendering
	// Bullet will figure out what needs to be drawn then call to
	// our DebugDrawer class to do the rendering for us
	m_pWorld->debugDrawWorld();
}

void Simulator::UpdateCamera(){
	// exit in erroneous situations
	if (m_screenWidth == 0 && m_screenHeight == 0)
		return;

	// select the projection matrix
	glMatrixMode(GL_PROJECTION);
	// set it to the matrix-equivalent of 1
	glLoadIdentity();
	// determine the aspect ratio of the screen
	float aspectRatio = m_screenWidth / (float)m_screenHeight;
	// create a viewing frustum based on the aspect ratio and the
	// boundaries of the camera
	glFrustum(-aspectRatio * m_nearPlane, aspectRatio * m_nearPlane, -m_nearPlane, m_nearPlane, m_nearPlane, m_farPlane);
	// the projection matrix is now set

	// select the view matrix
	glMatrixMode(GL_MODELVIEW);
	// set it to '1'
	glLoadIdentity();

	// create a view matrix based on the camera's position and where it's
	// looking
	gluLookAt(0, 450, 0, 0, 0, 0, 0, 0, 1);
	glTranslatef(-trans_x, distance, trans_y);
	glRotatef((float)rot_y + (float)record_y, -1.0, 0.0, 0.0);   //以x軸當旋轉軸 
	glRotatef((float)rot_x + (float)record_x, 0.0, 0.0, 1.0);   //以z軸當旋轉軸 
	glTranslatef(-360, 0, -180);
	// the view matrix is now set
}

void Simulator::Mouse(int button, int state, int x, int y){
	switch (button){
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_UP)
		{
			record_x += x - old_rot_x;
			record_y += y - old_rot_y;

			rot_x = 0;   //沒有歸零會有不理想的結果 
			rot_y = 0;

			currentButton = GLUT_TOT_BUTTON;
		}
		else // GLUT_DOWN
		{
			old_rot_x = x;
			old_rot_y = y;
			currentButton = GLUT_LEFT_BUTTON;
		}
		break;
	case GLUT_MIDDLE_BUTTON:
		if (state == GLUT_UP)
		{
			currentButton = GLUT_TOT_BUTTON;
		}
		else // GLUT_DOWN
		{
			old_dis_y = y;
			currentButton = GLUT_MIDDLE_BUTTON;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_UP)
		{
			currentButton = GLUT_TOT_BUTTON;
		}
		else // GLUT_DOWN
		{
			currentButton = GLUT_RIGHT_BUTTON;
			old_tran_x = x;
			old_tran_y = y;
		}
		break;
	}
}

void Simulator::PassiveMotion(int x, int y){

}

void Simulator::Motion(int x, int y){
	switch (currentButton){
	case GLUT_LEFT_BUTTON:
		rot_x = x - old_rot_x;
		rot_y = y - old_rot_y;
		break;
	case GLUT_MIDDLE_BUTTON:
		distance -= 0.3*(y - old_dis_y);
		old_dis_y = y;
		break;
	case GLUT_RIGHT_BUTTON:
		trans_x += 0.3*(x - old_tran_x);
		trans_y -= 0.3*(y - old_tran_y);
		old_tran_x = x;
		old_tran_y = y;
		break;
	}
	glutPostRedisplay();
}

void Simulator::Display(){

}

void Simulator::Initialize(){
	// this function is called inside glutmain() after
	// creating the window, but before handing control
	// to FreeGLUT
	// create some floats for our ambient, diffuse, specular and position
	GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f }; // dark grey
	GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat position[] = { 100.0f, 500.0f, 1000.0f, 0.0f };

	// set the ambient, diffuse, specular and position for LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glEnable(GL_LIGHTING); // enables lighting
	glEnable(GL_LIGHT0); // enables the 0th light
	glEnable(GL_COLOR_MATERIAL); // colors materials when lighting is enabled

	// enable specular lighting via materials
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMateriali(GL_FRONT, GL_SHININESS, 15);

	// enable smooth shading
	glShadeModel(GL_SMOOTH);

	// enable depth testing to be 'less than'
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	// set the backbuffer clearing color to a lightish blue
	glClearColor(0.6, 0.65, 0.85, 0);

	// initialize the physics system
	InitializeBullet();

	InitRobotEnv();
}

void Simulator::DrawBox(const btVector3 &halfSize){
	float halfWidth = halfSize.x();
	float halfHeight = halfSize.y();
	float halfDepth = halfSize.z();

	// create the vertex positions
	btVector3 vertices[8] = {
		btVector3(halfWidth, halfHeight, halfDepth),
		btVector3(-halfWidth, halfHeight, halfDepth),
		btVector3(halfWidth, -halfHeight, halfDepth),
		btVector3(-halfWidth, -halfHeight, halfDepth),
		btVector3(halfWidth, halfHeight, -halfDepth),
		btVector3(-halfWidth, halfHeight, -halfDepth),
		btVector3(halfWidth, -halfHeight, -halfDepth),
		btVector3(-halfWidth, -halfHeight, -halfDepth) };

	// create the indexes for each triangle, using the 
	// vertices above. Make it static so we don't waste 
	// processing time recreating it over and over again
	static int indices[36] = {
		0, 1, 2,
		3, 2, 1,
		4, 0, 6,
		6, 0, 2,
		5, 1, 4,
		4, 1, 0,
		7, 3, 1,
		7, 1, 5,
		5, 4, 7,
		7, 4, 6,
		7, 2, 3,
		7, 6, 2 };

	// start processing vertices as triangles
	glBegin(GL_TRIANGLES);

	// increment the loop by 3 each time since we create a 
	// triangle with 3 vertices at a time.

	for (int i = 0; i < 36; i += 3) {
		// get the three vertices for the triangle based
		// on the index values set above
		// use const references so we don't copy the object
		// (a good rule of thumb is to never allocate/deallocate
		// memory during *every* render/update call. This should 
		// only happen sporadically)
		const btVector3 &vert1 = vertices[indices[i]];
		const btVector3 &vert2 = vertices[indices[i + 1]];
		const btVector3 &vert3 = vertices[indices[i + 2]];

		// create a normal that is perpendicular to the 
		// face (use the cross product)
		btVector3 normal = (vert2 - vert1).cross(vert3 - vert1);
		normal.normalize();

		// set the normal for the subsequent vertices
		glNormal3f(normal.getX(), normal.getY(), normal.getZ());

		// create the vertices
		glVertex3f(vert1.x(), vert1.y(), vert1.z());
		glVertex3f(vert2.x(), vert2.y(), vert2.z());
		glVertex3f(vert3.x(), vert3.y(), vert3.z());
	}

	// stop processing vertices
	glEnd();
}

void Simulator::DrawSphere(const btScalar &radius){
	// some constant values for more many segments to build the sphere from
	static int lateralSegments = 25;
	static int longitudinalSegments = 25;

	// iterate laterally
	for (int i = 0; i <= lateralSegments; i++) {
		// do a little math to find the angles of this segment
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar)(i - 1) / lateralSegments);
		btScalar z0 = radius*sin(lat0);
		btScalar zr0 = radius*cos(lat0);

		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar)i / lateralSegments);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);

		// start rendering strips of quads (polygons with 4 poins)
		glBegin(GL_QUAD_STRIP);
		// iterate longitudinally
		for (int j = 0; j <= longitudinalSegments; j++) {
			// determine the points of the quad from the lateral angles
			btScalar lng = 2 * SIMD_PI * (btScalar)(j - 1) / longitudinalSegments;
			btScalar x = cos(lng);
			btScalar y = sin(lng);
			// draw the normals and vertices for each point in the quad
			// since it is a STRIP of quads, we only need to add two points
			// each time to create a whole new quad

			// calculate the normal
			btVector3 normal = btVector3(x*zr0, y*zr0, z0);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the first vertex
			glVertex3f(x * zr0, y * zr0, z0);

			// calculate the next normal
			normal = btVector3(x*zr1, y*zr1, z1);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the second vertex
			glVertex3f(x * zr1, y * zr1, z1);

			// and repeat...
		}
		glEnd();
	}
}

void Simulator::DrawCylinder(const btScalar &radius, const btScalar &halfHeight){
	static int slices = 15;
	static int stacks = 10;
	// tweak the starting position of the
	// cylinder to match the physics object
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.0, -halfHeight);
	// create a quadric object to render with
	GLUquadricObj *quadObj = gluNewQuadric();
	// set the draw style of the quadric
	gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
	gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
	// create a disk to cap the cylinder
	gluDisk(quadObj, 0, radius, slices, stacks);
	// create the main hull of the cylinder (no caps)
	gluCylinder(quadObj, radius, radius, 2.f*halfHeight, slices, stacks);
	// shift the position and rotation again
	glTranslatef(0.0f, 0.0f, 2.f*halfHeight);
	glRotatef(-180.0f, 0.0f, 1.0f, 0.0f);
	// draw the cap on the other end of the cylinder
	gluDisk(quadObj, 0, radius, slices, stacks);
	// don't need the quadric anymore, so remove it
	// to save memory
	gluDeleteQuadric(quadObj);
}

void Simulator::DrawConvexHull(const btCollisionShape* shape){
	// get the polyhedral data from the convex hull
	const btConvexPolyhedron* pPoly = shape->isPolyhedral() ? ((btPolyhedralConvexShape*)shape)->getConvexPolyhedron() : 0;
	if (!pPoly) return;

	// begin drawing triangles
	glBegin(GL_TRIANGLES);

	// iterate through all faces
	for (int i = 0; i < pPoly->m_faces.size(); i++) {
		// get the indices for the face
		int numVerts = pPoly->m_faces[i].m_indices.size();
		if (numVerts>2)	{
			// iterate through all index triplets
			for (int v = 0; v <pPoly->m_faces[i].m_indices.size() - 2; v++) {
				// grab the three vertices
				btVector3 v1 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[0]];
				btVector3 v2 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v + 1]];
				btVector3 v3 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v + 2]];
				// calculate the normal
				btVector3 normal = (v3 - v1).cross(v2 - v1);
				normal.normalize();
				// draw the triangle
				glNormal3f(normal.getX(), normal.getY(), normal.getZ());
				glVertex3f(v1.x(), v1.y(), v1.z());
				glVertex3f(v2.x(), v2.y(), v2.z());
				glVertex3f(v3.x(), v3.y(), v3.z());
			}
		}
	}
	// done drawing
	glEnd();
}

void Simulator::DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color){
	static btConcaveShape* concaveMesh;
	static GlDrawcallback drawCallback;
	// set the color
	glColor3f(color.x(), color.y(), color.z());

	// push the matrix stack
	glPushMatrix();
	glMultMatrixf(transform);

	// make a different draw call based on the object type
	switch (pShape->getShapeType()) {
		// an internal enum used by Bullet for boxes
	case BOX_SHAPE_PROXYTYPE:
	{
								// assume the shape is a box, and typecast it
								const btBoxShape* box = static_cast<const btBoxShape*>(pShape);
								// get the 'halfSize' of the box
								btVector3 halfSize = box->getHalfExtentsWithMargin();
								// draw the box
								DrawBox(halfSize);
								break;
	}

	case SPHERE_SHAPE_PROXYTYPE:
	{
								   // assume the shape is a sphere and typecast it
								   const btSphereShape* sphere = static_cast<const btSphereShape*>(pShape);
								   // get the sphere's size from the shape
								   float radius = sphere->getMargin();
								   // draw the sphere
								   DrawSphere(radius);
								   break;
	}

	case CYLINDER_SHAPE_PROXYTYPE:
	{
									 // assume the object is a cylinder
									 const btCylinderShape* pCylinder = static_cast<const btCylinderShape*>(pShape);
									 // get the relevant data
									 float radius = pCylinder->getRadius();
									 float halfHeight = pCylinder->getHalfExtentsWithMargin()[1];
									 // draw the cylinder
									 DrawCylinder(radius, halfHeight);

									 break;
	}

	case CONVEX_HULL_SHAPE_PROXYTYPE:
	{
										// draw the convex hull shape...whatever it is
										DrawConvexHull(pShape);
										break;
	}

	case COMPOUND_SHAPE_PROXYTYPE:
	{
									 // get the shape
									 const btCompoundShape* pCompound = static_cast<const btCompoundShape*>(pShape);
									 // iterate through the children
									 for (int i = 0; i < pCompound->getNumChildShapes(); ++i) {
										 // get the transform of the sub-shape
										 btTransform thisTransform = pCompound->getChildTransform(i);
										 btScalar thisMatrix[16];
										 thisTransform.getOpenGLMatrix(thisMatrix);
										 // call drawshape recursively for each child. The matrix
										 // stack takes care of positioning/orienting the object for us
										 DrawShape(thisMatrix, pCompound->getChildShape(i), color);
									 }
									 break;
	}
	
	case GIMPACT_SHAPE_PROXYTYPE:
		concaveMesh = (btConcaveShape*)pShape;
		m_pWorld->getBroadphase()->getBroadphaseAabb(aabbMin, aabbMax);
		aabbMin -= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		aabbMax += btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		drawCallback.m_wireframe = (0 & btIDebugDraw::DBG_DrawWireframe) != 0;
		concaveMesh->processAllTriangles(&drawCallback, aabbMin, aabbMax);
		break;

	default:
		// unsupported type
		break;
	}

	// pop the stack
	glPopMatrix();
}

void Simulator::InitializeBullet(){
	// create the collision configuration
	m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the world
	m_pWorld = new btCollisionWorld(m_pDispatcher, m_pBroadphase, m_pCollisionConfiguration);
	// create the debug drawer
	m_pDebugDrawer = new DebugDrawer();
	// set the initial debug level to 0
	m_pDebugDrawer->setDebugMode(0);
	// add the debug drawer to the world
	m_pWorld->setDebugDrawer(m_pDebugDrawer);
}

void Simulator::ShutdownBullet(){
	delete m_pWorld;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;
}

Object* Simulator::CreateObject(btCollisionShape* pShape, const btVector3 &color, const Eigen::Matrix4d &transMat, short int group, short int mask){
	// create new object
	Object* pObject = new Object(pShape, color, transMat);
	// push it to the list
	m_objects.push_back(pObject);

	if (m_pWorld){
		m_pWorld->addCollisionObject(pObject->getCollisionObject(), group, mask);
	}
	return pObject;
}

btCollisionShape* Simulator::loadObjFile(char* filename){
	ConvexDecomposition::WavefrontObj wo;
	wo.loadObj(filename);

	// 做接口
	int* gIndices;
	gIndices = new int[wo.mTriCount*3];
	for (int i = 0, n = wo.mTriCount*3; i < n; ++i){
		gIndices[i] = wo.mIndices[i];
	}
	btScalar* gVertices;
	gVertices = new btScalar[wo.mVertexCount*3];
	for (int i = 0, n = wo.mVertexCount*3; i < n; ++i){
		gVertices[i] = wo.mVertices[i];
	}
	// 這兩個接口不能release掉!! 因為bullet Shape根據的指標是指這裡!!

	// create trimesh
	btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(wo.mTriCount, &gIndices[0], 3 * sizeof(int), wo.mVertexCount, gVertices, sizeof(btScalar)*3);
	btGImpactMeshShape * trimesh = new btGImpactMeshShape(indexVertexArrays);
	trimesh->setLocalScaling(btVector3(1.f, 1.f, 1.f));
	trimesh->updateBound();

	//register algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(m_pDispatcher);

	return trimesh;
}

void Simulator::CheckForCollisionEvents(){
	//Perform collision detection // 這行一定要有!
	m_pWorld->performDiscreteCollisionDetection();
	int numManifolds = m_pWorld->getDispatcher()->getNumManifolds();
	//For each contact manifold
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = m_pWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)contactManifold->getBody0();
		btCollisionObject* obB = (btCollisionObject*)contactManifold->getBody1();
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		int numContacts = contactManifold->getNumContacts();
		//For each contact point in that manifold
		for (int j = 0; j < numContacts; j++) {
			//Get the contact information
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();
			double ptdist = pt.getDistance();
			// TODO
			if (ptdist < _contactEps)
				for (auto& it : m_objects)
					if (it->getCollisionObject() == obA || it->getCollisionObject() == obB)
						it->setContact(true);
		}
	}

}

void Simulator::InitRobotEnv(){
	const double pi = acos(-1.0);
	
	{
		_arm = new RobotArm();
		double d[6] = { 0, 0, 371, 0, 280, 0 }; // d[0] = 1220 平台 沒加上去
		// a[5] 到手掌心 所以跟原來黑金剛不一樣
		double a[6] = { 0, 0, 10, -10, 0, 96 };
		double alpha[6] = { pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2 };
		double theta[6] = { 0, pi, pi, -pi / 2, pi, pi / 2 };
		double maxlimits[6] = { 5 * pi / 9, pi / 2, 115 * pi / 180, pi / 2, 115 * pi / 180, 85 * pi / 180 };
		double minlimits[6] = { -5 * pi / 9, -pi / 2, -115 * pi / 180, -40 * pi / 180, -115 * pi / 180, -85 * pi / 180 };
		for (int i = 0; i < 6; ++i){
			_arm->addRevoluteFrame(a[i], alpha[i], d[i], theta[i], minlimits[i], maxlimits[i], i);
		} // id1 ~ 6
		_arm->searchEE();

		// add base obj
		_arm->getAllFrameList()[0]->setObject(CreateObject(loadObjFile("model/arm/base.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->getAllFrameList()[1]->setObject(CreateObject(loadObjFile("model/arm/0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->getAllFrameList()[2]->setObject(CreateObject(loadObjFile("model/arm/1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->getAllFrameList()[3]->setObject(CreateObject(loadObjFile("model/arm/2.obj"), btVector3(0.0, 0.0, 0.0), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->getAllFrameList()[4]->setObject(CreateObject(loadObjFile("model/arm/3.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->getAllFrameList()[5]->setObject(CreateObject(loadObjFile("model/arm/4.obj"), btVector3(0.0, 0.0, 0.0), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->getAllFrameList()[6]->setObject(CreateObject(loadObjFile("model/arm/5.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_ARM, COLLISIONGROUP_ENV));
		_arm->setObjectPose();
	}
	
	{
		_hand = new RobotHand();
		Matrix4d setRobotHandTrans;
		// 由於將原點移至手掌心
		// z減掉69
		// Thumb
		setRobotHandTrans << 1, 0, 0, 3.4,
			0, 1, 0, -22.38,
			0, 0, 1, -16.76,
			0, 0, 0, 1;
		_hand->addBasicFrame(setRobotHandTrans, 0); // id1
		_hand->addRevoluteFrame(50.74, pi / 2, 13.08, -pi / 2, 0, pi / 2, 1); // id2
		_hand->addRevoluteFrame(38.95, 0, 0, pi / 4, 0, pi / 2, 2); // id3
		_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 3); // id4
		_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 4, 4); // id5

		// Index
		setRobotHandTrans << 0, 0, 1, 12.05,
			1, 0, 0, -42.25,
			0, 1, 0, 11.59,
			0, 0, 0, 1;
		_hand->addBasicFrame(setRobotHandTrans, 0); // id6
		_hand->addRevoluteFrame(44.5, pi / 2, -9.65, pi / 2, 0, pi / 12, 6); // id7
		_hand->addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 7); // id8
		_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, pi * 99 / 180, 8); // id9
		_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 9, 9); // id10

		// Middle
		setRobotHandTrans << 0, 1, 0, 2.4,
			0, 0, 1, -17.4,
			1, 0, 0, 69.22,
			0, 0, 0, 1;
		_hand->addBasicFrame(setRobotHandTrans, 0); // id11
		_hand->addRevoluteFrame(38.88, 0, 0, 0, 0, pi / 2, 11); // id12
		_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, pi * 99 / 180, 12); // id13
		_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 13, 13); // id14

		// Ring
		setRobotHandTrans << 0, 0, -1, 12.05,
			-1, 0, 0, 5.75,
			0, 1, 0, 11.59,
			0, 0, 0, 1;
		_hand->addBasicFrame(setRobotHandTrans, 0); // id15
		_hand->addPassiveFrame(44.5, -pi / 2, 9.65, pi / 2, 1, 7, 15); // id16
		_hand->addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 16); // id17
		_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 17); // id18
		_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 18, 18); // id19

		// Pinky
		setRobotHandTrans << 0, 0, -1, 12.05,
			-1, 0, 0, 29.75,
			0, 1, 0, -1.39,
			0, 0, 0, 1;
		_hand->addBasicFrame(setRobotHandTrans, 0); // id20
		_hand->addPassiveFrame(44.5, -pi / 2, 9.65, pi / 2, 21.5 / 15, 7, 20); // id21
		_hand->addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 21); // id22
		_hand->addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 22); // id23
		_hand->addPassiveFrame(21.6, 0, 0, 0, 0.82, 23, 23); // id24
		_hand->searchEE();

		// Base
		_hand->getAllFrameList()[0]->setObject(CreateObject(loadObjFile("model/hand/base_v2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_HANDBASE, COLLISIONGROUP_ENV));

		// Thumb
		_hand->getAllFrameList()[2]->setObject(CreateObject(loadObjFile("model/hand/thumb_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[3]->setObject(CreateObject(loadObjFile("model/hand/thumb_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[4]->setObject(CreateObject(loadObjFile("model/hand/thumb_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[5]->setObject(CreateObject(loadObjFile("model/hand/thumb_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_THUMB, COLLISIONGROUP_FINGER | COLLISIONGROUP_ENV));

		// Index
		_hand->getAllFrameList()[7]->setObject(CreateObject(loadObjFile("model/hand/index_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[8]->setObject(CreateObject(loadObjFile("model/hand/index_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[9]->setObject(CreateObject(loadObjFile("model/hand/index_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[10]->setObject(CreateObject(loadObjFile("model/hand/index_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));

		// middle
		_hand->getAllFrameList()[12]->setObject(CreateObject(loadObjFile("model/hand/middle_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[13]->setObject(CreateObject(loadObjFile("model/hand/middle_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[14]->setObject(CreateObject(loadObjFile("model/hand/middle_2.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_THUMB | COLLISIONGROUP_ENV));

		// ring
		_hand->getAllFrameList()[16]->setObject(CreateObject(loadObjFile("model/hand/ring_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[17]->setObject(CreateObject(loadObjFile("model/hand/ring_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[18]->setObject(CreateObject(loadObjFile("model/hand/ring_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[19]->setObject(CreateObject(loadObjFile("model/hand/ring_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));

		// pinky
		_hand->getAllFrameList()[21]->setObject(CreateObject(loadObjFile("model/hand/pinky_0.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[22]->setObject(CreateObject(loadObjFile("model/hand/pinky_1.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[23]->setObject(CreateObject(loadObjFile("model/hand/pinky_2.obj"), btVector3(0.4, 0.4, 0.4), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));
		_hand->getAllFrameList()[24]->setObject(CreateObject(loadObjFile("model/hand/pinky_3.obj"), btVector3(0.0, 0.25, 0.25), Matrix4d::Identity(), COLLISIONGROUP_FINGER, COLLISIONGROUP_ENV));

		_hand->setObjectPose();
		_hand->setFingerRelation();
	}

	_hand->connectToArm(_arm);
}
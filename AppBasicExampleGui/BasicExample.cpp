/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#include <vector>
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Z 1

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	btRigidBody* t;
	btVector3 p[9999999];
	int count=0;
	virtual ~BasicExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 3;
		float pitch = -15;
		float yaw = 170;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
	
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	}

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -5, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	//wedge
	{
	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(2.)));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -0.3, 0));
	groundTransform.setRotation(btQuaternion(btVector3(2, 1, 0), SIMD_PI * 0.1));
	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setFriction(.4);
	body->getAngularVelocity();
	body->getLinearVelocity();



	//add the body to the dynamics world
	m_dynamicsWorld->addRigidBody(body);
}








	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);


			for (int k = 0; k < ARRAY_SIZE_Y; k++)
			{
				for (int i = 0; i < ARRAY_SIZE_X; i++)
				{
					for (int j = 0; j < ARRAY_SIZE_Z; j++)
					{
						startTransform.setOrigin(btVector3(
							btScalar(-0.2),
							btScalar(2 + .2),
							btScalar(-0.9)));
						this->t = createRigidBody(mass, startTransform, colShape);

					}
				}
			}

	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

}

void BasicExample::renderScene()
{
			
	//char u[6] = "Hello";
	//const char* teext = &u[0];
	//m_guiHelper->drawText3D(teext, 0.0f, 4.0f, 0.0f, 2.0f);

	for (float i = 1; i < 10; i += 0.5)
	{
		if (m_dynamicsWorld->getDebugDrawer())
		{
			m_dynamicsWorld->getDebugDrawer()->drawArc(btVector3(0, 0.002, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), btScalar(i/2), btScalar(i/2), btScalar(0), btScalar(SIMD_2_PI), btVector3(0, 0, 1), false);
		}
	}
	CommonRigidBodyBase::renderScene();

	btTransform trans= this->t->getWorldTransform();

	btVector3 objectposition = trans.getOrigin();

	int c = this->count;
	this->p[c] = objectposition;
	this->count = this->count + 1;
	char performance[50];
	sprintf(performance, "Position = %.2f %.2f %.2f", objectposition.x(), objectposition.y(), objectposition.z());
	m_guiHelper->drawText3D(performance, objectposition.x(), objectposition.y() + 0.2, objectposition.z(), 1);
	for (int i = 1; i < this->count; i++)
	{
		if (this->count == 0)
		{
			m_dynamicsWorld->getDebugDrawer()->drawLine(btVector3(-0.2, 2.2, -0.9), this->p[i], btVector3(1, 0, 0));
		}
		else
		{
			m_dynamicsWorld->getDebugDrawer()->drawLine(this->p[i], this->p[i-1], btVector3(1, 0, 0));
		}
	}
	
}








CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)

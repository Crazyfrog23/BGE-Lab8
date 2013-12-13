#include "EoinsAdditions.h"
#include "Game.h"
#include "Sphere.h"
#include "Box.h"
#include "Cylinder.h"
#include "Ground.h"
#include "Content.h"
#include "PhysicsCamera.h"
#include "Model.h"
#include "dirent.h"
#include "Utils.h"
#include <SOIL.h>
#include <ctime>
using namespace BGE;

EoinsAdditions::EoinsAdditions()
{
}


EoinsAdditions::~EoinsAdditions(void)
{
}

shared_ptr<PhysicsController> EoinsAdditions::NewCreateBox(float width, float height, float depth, glm::vec3 pos, glm::quat quat, int massIn, float margin, int friction, int restitution, bool kinimatic, btDiscreteDynamicsWorld * dynamicsWorld, string tagIn)
{
	// Create the shape
	btCollisionShape * boxShape = new btBoxShape(btVector3(width, height, depth) * 0.5);
	if(margin != 0)
	{
		boxShape->setMargin(margin);
	}
	btScalar mass = massIn;
	btVector3 boxInertia(0,0,0);
	boxShape->calculateLocalInertia(mass,boxInertia);

	// This is a container for the box model
	shared_ptr<Box> box = make_shared<Box>(width, height, depth);
	box->worldMode = GameComponent::from_child;
	box->position = pos;
	Game::Instance()->Attach(box);

	// Create the rigid body
	btDefaultMotionState * boxMotionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));			
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,  boxMotionState, boxShape, boxInertia);
	//fallRigidBodyCI.m_restitution = 1;
	//fallRigidBodyCI.m_linearDamping = 0.2f;
	//fallRigidBodyCI.m_angularDamping = 0.1f;
	//fallRigidBodyCI.m_friction = 0.0f;
	btRigidBody * body = new btRigidBody(fallRigidBodyCI);

	if(friction != 0)
	{
		body->setFriction(friction);
	}
	if(restitution != 0)
	{
		body->setRestitution(restitution);
	}

	dynamicsWorld->addRigidBody(body);

	// Create the physics component and add it to the box
	shared_ptr<PhysicsController> boxController = make_shared<PhysicsController>(PhysicsController(boxShape, body, boxMotionState));
	boxController->tag = tagIn;
	body->setUserPointer(boxController.get());
	if(kinimatic == true)
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setActivationState(DISABLE_DEACTIVATION);
	}
	else
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	}
	
	box->Attach(boxController);

	return boxController;
}

void EoinsAdditions::CreateWall(glm::vec3 startAt, float width, float height, btDiscreteDynamicsWorld * dynamicsWorld, float blockWidth, float blockHeight, float blockDepth)
{
	float z = startAt.z;
	float gap = 0;

	for (int w = 0 ; w < width ; w ++)
	{
		for (int h = 0 ; h < height ; h ++)	
		{
			float x = startAt.x + ((blockWidth + 2) * w);
			float y = ((blockHeight + gap) / 2.0f) + ((blockHeight + gap) * h);
			NewCreateBox(blockWidth, blockHeight, blockDepth, glm::vec3(x, y, z), glm::quat(),1,0,0,0,false,dynamicsWorld,"Wall");
		}
	}
}

void EoinsAdditions::CreatePyramid(float width, float height, float depth, glm::vec3 centre, glm::quat quat, int towerHeight, int towerWidth,btDiscreteDynamicsWorld * dynamicsWorld)
{
	glm::vec3 pos;

	for(int i = 0; i < towerHeight; i++)
	{
		pos.y = centre.y + (height * i);

		for(int j = 0;j<towerWidth;j++)
		{
			pos.x = (centre.x - (towerWidth / 2)) + (j * width);
			for(int k = 0;k<towerWidth;k++)
			{
				pos.z = (centre.z - (towerWidth / 2)) + (k * width);
				NewCreateBox(width, height, depth, glm::vec3(pos.x, pos.y, pos.z), glm::quat(),1,0,0,0,false,dynamicsWorld,"Pyramid");
			}
		}

		towerWidth = towerWidth - 2;
	}
}

void EoinsAdditions::CreateRagDoll(glm::vec3 scale, glm::vec3 pos, glm::quat quat, shared_ptr<PhysicsFactory> phyFactory,btDiscreteDynamicsWorld * dynamicsWorld)
{
	shared_ptr<PhysicsController> body = NewCreateBox(0.6f, 2, 1, glm::vec3(pos.x, pos.y, pos.z), glm::quat(),1,0,0,0,false,dynamicsWorld,"Body");
	shared_ptr<PhysicsController> rUpperArm = NewCreateBox(0.5f, 0.5f, 2, glm::vec3(pos.x, pos.y+0.5, pos.z+1.5f), glm::quat(),1,0,0,0,false,dynamicsWorld,"RArm");
	shared_ptr<PhysicsController> lUpperArm = NewCreateBox(0.5f, 0.5f, 2, glm::vec3(pos.x, pos.y+0.5, pos.z-1.5f), glm::quat(),1,0,0,0,false,dynamicsWorld,"LArm");
	shared_ptr<PhysicsController> rLeg = NewCreateBox(0.5f, 2, 0.5f, glm::vec3(pos.x, pos.y-2, pos.z+0.25f), glm::quat(),1,0,0,0,false,dynamicsWorld,"RLeg");
	shared_ptr<PhysicsController> lLeg = NewCreateBox(0.5f, 2, 0.5f, glm::vec3(pos.x, pos.y-2, pos.z-0.25f), glm::quat(),1,0,0,0,false,dynamicsWorld,"LLeg");
	shared_ptr<PhysicsController> head = phyFactory->CreateSphere(0.5,glm::vec3(pos.x, pos.y+1.5, pos.z), glm::quat());

	
	btPoint2PointConstraint * ptpConstraint = new btPoint2PointConstraint(*body->rigidBody, *rUpperArm->rigidBody, btVector3(-0.2f,-0.75f,0.75f),btVector3(0,0,1));
	dynamicsWorld->addConstraint(ptpConstraint);
	ptpConstraint = new btPoint2PointConstraint(*body->rigidBody, *lUpperArm->rigidBody, btVector3(-0.2f,-0.75f,-0.75f),btVector3(0,0,1));
	dynamicsWorld->addConstraint(ptpConstraint);
	ptpConstraint = new btPoint2PointConstraint(*body->rigidBody, *rLeg->rigidBody, btVector3(0,1.2f,0.3f),btVector3(0,1,0));
	dynamicsWorld->addConstraint(ptpConstraint);
	ptpConstraint = new btPoint2PointConstraint(*body->rigidBody, *lLeg->rigidBody, btVector3(0,1.2f,-0.3f),btVector3(0,1,0));
	dynamicsWorld->addConstraint(ptpConstraint);
	ptpConstraint = new btPoint2PointConstraint(*body->rigidBody, *head->rigidBody, btVector3(0,-1.2,0),btVector3(0,1,0));
	dynamicsWorld->addConstraint(ptpConstraint);

}

void EoinsAdditions::CreateSnookerTable(float width, float height, glm::vec3 centre, glm::quat quat, shared_ptr<PhysicsFactory> phyFactory,btDiscreteDynamicsWorld * dynamicsWorld)
{
	centre.y = 1;
	//shared_ptr<PhysicsController> TableBody = NewCreateBox(width, 10,height, glm::vec3(centre.x, centre.y, centre.z), glm::quat(),1,true,dynamicsWorld);


	shared_ptr<PhysicsController> Ball = phyFactory->CreateSphere(3, glm::vec3(centre.x, centre.y, centre.z), glm::quat());

	shared_ptr<PhysicsController> Bumper1 = NewCreateBox(10, 15, height-40, glm::vec3(centre.x + (width/2)-6, centre.y+10, centre.z), glm::quat(),1,1,0,2,true,dynamicsWorld,"NearWall");
	shared_ptr<PhysicsController> Bumper2 = NewCreateBox(10, 15, height, glm::vec3(centre.x - (width/2)-5, centre.y+5, centre.z), glm::quat(),1,1,0,2,true,dynamicsWorld,"FarWall");
	shared_ptr<PhysicsController> Bumper3 = NewCreateBox(width, 15, 10, glm::vec3(centre.x, centre.y+5, centre.z + (height/2)-6), glm::quat(),1,1,0,2,true,dynamicsWorld,"LeftWall");
	shared_ptr<PhysicsController> Bumper4 = NewCreateBox(width, 15, 10, glm::vec3(centre.x, centre.y+5, centre.z - (height/2)+6), glm::quat(),1,1,0,2,true,dynamicsWorld,"RightWall");

}

shared_ptr<PhysicsController> EoinsAdditions::CreateBreakout(float width, float height, glm::vec3 centre, glm::quat quat, shared_ptr<PhysicsFactory> phyFactory,btDiscreteDynamicsWorld * dynamicsWorld)
{
	centre.y = 1;
	//shared_ptr<PhysicsController> TableBody = NewCreateBox(width, 10,height, glm::vec3(centre.x, centre.y, centre.z), glm::quat(),1,true,dynamicsWorld);

	//create paddle
	shared_ptr<PhysicsController> Bumper1 = NewCreateBox(5, 15, 20, glm::vec3(centre.x + (width/2)-5, centre.y+5, centre.z), glm::quat(),1,1,0,2,true,dynamicsWorld,"Paddle");
	//create bumpers
	shared_ptr<PhysicsController> Bumper2 = NewCreateBox(10, 15, height, glm::vec3(centre.x - (width/2)-5, centre.y+5, centre.z), glm::quat(),1,1,0,2,true,dynamicsWorld,"FarWall");

	glm::vec3 axis = glm::cross(GameComponent::basisRight, GameComponent::basisLook);
    axis = glm::normalize(axis);
    float theta = -4.0f;//glm::acos(glm::dot(GameComponent::basisUp, GameComponent::basisLook));
	glm::quat bumper3Quat = glm::angleAxis(theta, axis);
	shared_ptr<PhysicsController> Bumper3 = NewCreateBox(width, 15, 10, glm::vec3(centre.x, centre.y+5, centre.z + (height/2)+2), bumper3Quat,1,1,0,2,true,dynamicsWorld,"LeftWall");

	axis = glm::cross(GameComponent::basisRight, GameComponent::basisLook);
    axis = glm::normalize(axis);
    theta = 4.0f;//glm::acos(glm::dot(GameComponent::basisUp, GameComponent::basisLook));
	glm::quat bumper4Quat = glm::angleAxis(theta, axis);
	shared_ptr<PhysicsController> Bumper4 = NewCreateBox(width, 15, 10, glm::vec3(centre.x, centre.y+5, centre.z - (height/2)-2), bumper4Quat,1,1,0,2,true,dynamicsWorld,"RightWall");

	//these are safety bumpers to stop the paddle from bouncing off the normal bumpers
	//shared_ptr<PhysicsController> Bumper5 = NewCreateBox(10, 15, 1, glm::vec3(centre.x + (width/2)-5, centre.y+5, centre.z + (height/2)-17), glm::quat(),1,1,0,0,true,dynamicsWorld);
	//shared_ptr<PhysicsController> Bumper6 = NewCreateBox(10, 15, 1, glm::vec3(centre.x + (width/2)-5, centre.y+5, centre.z - (height/2)+17), glm::quat(),1,1,0,0,true,dynamicsWorld);


	// this was utilised when the bumper was not set to a rigid body
	//Bumper1->rigidBody->setDamping(btScalar(0.95),btScalar(0));

	return Bumper1;
}

void EoinsAdditions::TakeScreenShot()
{
	//get window resolution
	//this does not work as SOIL_save_screenshot will not accept variables without crashing, seems to be due to graphics card driver
	//may work on another computer
	/*
	GLint screenSize[4];
	glGetIntegerv(GL_VIEWPORT,screenSize);
	int screenWidth = screenSize[3];
	int screenHeight = screenSize[4];*/

	//get current time
	time_t t = time(0);
	std::ostringstream ScreenShotName;
	ScreenShotName << "Content/Screenshots/screenshot_" << t << ".png";
	string ScreenNameString = ScreenShotName.str();
	const char* ScreenNameChar = ScreenNameString.c_str();
	int save_result = SOIL_save_screenshot(ScreenNameChar,SOIL_SAVE_TYPE_BMP,0,0,1280,720);



	GLuint tex_2d = SOIL_load_OGL_texture("Content/img_test.png",SOIL_LOAD_AUTO,SOIL_CREATE_NEW_ID,SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	
	/* check for an error during the load process */
	if( 0 == tex_2d )
	{
		printf( "SOIL loading error: '%s'\n", SOIL_last_result() );
	}
}
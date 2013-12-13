#include "Breakout2D.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"
#include <gtc\random.hpp>
#include <fstream>

using namespace BGE;

Breakout2D::Breakout2D(void)
{
	physicsFactory = NULL;
	eoinsFactory = NULL;
	dynamicsWorld = NULL;
	broadphase = new btDbvtBroadphase();
	dispatcher = NULL;
	solver = NULL;
	fullscreen = false;
	paddleSpeed = NULL;
	ballSpeed = 1;
	CountDownTimer = 20000;
	gameStarted = false;
	currentHighScore = 0;
	movingLeft = true;
}

Breakout2D::~Breakout2D(void)
{
}

bool Breakout2D::Initialise() 
{
	// Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
 
    // The world.
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	broadphase = new btAxisSweep3(worldMin,worldMax);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-9,0));

	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	

	eoinsFactory->CreateWall(glm::vec3(-20,0,20), 5, 5, dynamicsWorld);

	//eoinsFactory->CreatePyramid(1,1,1, glm::vec3(50,5,50), glm::quat(),25,25);


	eoinsFactory->CreateRagDoll(glm::vec3(1,1,1),glm::vec3(0,1,-10), glm::quat(), physicsFactory, dynamicsWorld);

	//create the break out game
	breakoutPaddle = eoinsFactory->CreateBreakout(200,100,glm::vec3(100,1,100),glm::quat(), physicsFactory, dynamicsWorld);

	//the ball and the breaking blocks must be created outside the breakout game as you can only return one physics controller - breakoutPaddle , but we need the ball to be returned here
	//This could be achived by passing the ball by referene as an argumnet to createBreakout, but haven't figured out how to pass shared_ptr by reference
	//with more time and future research ball could be passed by reference so no set up is needed for the ball outside the funciton
	Ball = physicsFactory->CreateSphere(3, glm::vec3(100+50, 1, 100), glm::quat());
	Ball->rigidBody->setLinearVelocity(GLToBtVector(glm::vec3(-120,0,rand()%(40+40 + 1) - 40)));   
	Ball->rigidBody->activate();

	//first row
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-50, 6, 100), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-50, 6, 86), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe1"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-50, 6, 114), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe2"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-50, 6, 72), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe3"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-50, 6, 128), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe4"));

	//second row
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-60, 6, 100), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe5"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-60, 6, 86), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe6"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-60, 6, 114), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe7"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-60, 6, 72), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe8"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-60, 6, 128), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe9"));

	//third row
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-70, 6, 100), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe10"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-70, 6, 86), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe11"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-70, 6, 114), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe12"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-70, 6, 72), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe13"));
	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-70, 6, 128), glm::quat(),1,1,0,2,true,dynamicsWorld,"deleteMe14"));

	BlockCounter1.push_back(eoinsFactory->NewCreateBox(5,15,10, glm::vec3(100-30, 6, 100), glm::quat(),1,1,0,2,true,dynamicsWorld,"dontDeleteMe"));

	blockCount = 15;

	//get high score

	//ofstream scoreFile;
	//scoreFile.open("Content/HighScore.txt");
	ifstream file("Content/HighScore.txt");
	while(getline(file, currentHighScoreString))
	 {
		std::istringstream iss(currentHighScoreString);
		std::string token;
		while (iss >> token)
		{
			currentHighScore = atoi(token.c_str());
		}
	 }



	if (!Game::Initialise()) {
		return false;
	}

	//reset camera position now that it is initialised
	camera->GetController()->position = glm::vec3(250, 150, 98);

	//this was attempting to rotate the camera

	//glm::vec3 axis = glm::cross(GameComponent::basisUp,  glm::vec3(-0.65f,-0.75f,0));
    //axis = glm::normalize(axis);
    //float theta = glm::acos(glm::dot(glm::vec3(1,1,1), GameComponent::basisUp));
	//camera->GetController()->orientation = glm::angleAxis(glm::degrees(theta), axis);
	//camera->GetController()->look = glm::vec3(-0.64f,1,1);

	return true;
}

void BGE::Breakout2D::Update(float timeDelta)
{
	dynamicsWorld->stepSimulation(timeDelta,7);
	Game::Update(timeDelta);

	if (keyState[SDL_SCANCODE_LEFT] && breakoutPaddle->position.z < 143)
	{
		//This moves the paddle as a kinitic rigid body
		btTransform newTrans;
		
		//used for damping, but didn't feel right
		//paddleSpeed = 5;
		
		breakoutPaddle->rigidBody->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin() += btVector3(0,0,1);
		breakoutPaddle->rigidBody->getMotionState()->setWorldTransform(newTrans);

		//this moves the paddle when it is setup as a normal rigid body
		//paddleSpeed = 30;
		//breakoutPaddle->rigidBody->setLinearVelocity(GLToBtVector(glm::vec3(0,0,paddleSpeed)));    
		//breakoutPaddle->rigidBody->activate();
	}
	else if(keyState[SDL_SCANCODE_RIGHT] && breakoutPaddle->position.z > 57)
	{
		//This moves the paddle as a kinitic rigid body
		btTransform newTrans;

		//used for damping, but didn't feel right
		//paddleSpeed = -5;

		breakoutPaddle->rigidBody->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin() += btVector3(0,0,-1);
		breakoutPaddle->rigidBody->getMotionState()->setWorldTransform(newTrans);

		//this moves the paddle when it is setup as a normal rigid body
		//paddleSpeed = -30;
		//breakoutPaddle->rigidBody->setLinearVelocity(GLToBtVector(glm::vec3(0,0,paddleSpeed)));    
		//breakoutPaddle->rigidBody->activate();	
	}

	//this was going to be used for a 3d version of breakout but so far i have not had time to build this
	else if(keyState[SDL_SCANCODE_UP])
	{
		btTransform newTrans;

		breakoutPaddle->rigidBody->getMotionState()->getWorldTransform(newTrans);

		glm::vec3 axis = glm::cross(GameComponent::basisUp, GameComponent::basisRight);
		axis = glm::normalize(axis);
		float theta = -5.0f;//glm::acos(glm::dot(GameComponent::basisUp, GameComponent::basisLook));
		glm::quat paddleQuat = glm::angleAxis(theta, axis);

		btQuaternion paddleQuat1 = GLToBtQuat(paddleQuat);

		newTrans.setRotation(paddleQuat1*newTrans.getRotation());
		breakoutPaddle->rigidBody->getMotionState()->setWorldTransform(newTrans);
		newTrans.setIdentity();
	}

	//used for damping, but didn't feel right, could have refined it more, as this code is only very early basic, clipping into side wall
	//but didn't think it would improve the game much
	/*
	else
	{
		if(paddleSpeed>0)
		{
			btTransform newTrans;
			paddleSpeed--;
			breakoutPaddle->rigidBody->getMotionState()->getWorldTransform(newTrans);
			newTrans.getOrigin() += btVector3(0,0,1);
			breakoutPaddle->rigidBody->getMotionState()->setWorldTransform(newTrans);
		}
		else if(paddleSpeed<0)
		{
			btTransform newTrans;
			paddleSpeed++;
			breakoutPaddle->rigidBody->getMotionState()->getWorldTransform(newTrans);
			newTrans.getOrigin() += btVector3(0,0,-1);
			breakoutPaddle->rigidBody->getMotionState()->setWorldTransform(newTrans);
		}
	}*/


	//this is used to keep the ball at a constant pace
	glm::vec3 setBallVelocity = BtToGLVector(Ball->rigidBody->getLinearVelocity());
	setBallVelocity = glm::normalize(setBallVelocity);
	setBallVelocity *= ballSpeed;

	//cout << "X: " << setBallVelocity.x << "Z: " <<  setBallVelocity.z << endl;

	Ball->rigidBody->setLinearVelocity(GLToBtVector(setBallVelocity));


	//debug information
	//cout << breakoutPaddle->position.z << endl;


	//take a screenshot
	if(keyState[SDL_SCANCODE_F2])
	{
		eoinsFactory->TakeScreenShot();
	}

	//start game
	if(keyState[SDL_SCANCODE_F3])
	{
		ballSpeed = 100;
		gameStarted = true;
	}





	//get a new ball
	if(keyState[SDL_SCANCODE_F4])
	{
		Ball = physicsFactory->CreateSphere(3, glm::vec3(100+50, 1, 100), glm::quat());
		Ball->rigidBody->setLinearVelocity(GLToBtVector(glm::vec3(-120,0,rand()%(40+40 + 1) - 40)));   
		Ball->rigidBody->activate();
	}

	//while game is running update collsions of objects & score
	if(blockCount>0 && gameStarted)
	{
		UpdateCollisions();
		CountDownTimer--;
	}
	else if(gameStarted)
	{
		physicsFactory->CreateFromModel("monkey", glm::vec3(100,0,100), glm::quat(), glm::vec3(5,5,5));
		gameStarted = false;
	}

	PrintText("Press F2 to take a screenshot");
	PrintText("Press F3 to start the game");
	PrintText("Press F4 for a new ball //does not currently work");
	PrintText("Press Esc to quit");
	//PrintText("Your score: " + CountDownTimer);

	//just to make it more difficult im putting in a moving brick

	if(BlockCounter1[15]->position.z < 130 && gameStarted && movingLeft)
	{
		btTransform newTrans;
		
		BlockCounter1[15]->rigidBody->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin() += btVector3(0,0,1);
		

		glm::vec3 axis = glm::cross(GameComponent::basisRight, GameComponent::basisLook);
		axis = glm::normalize(axis);
		float theta = -5.0f;//glm::acos(glm::dot(GameComponent::basisUp, GameComponent::basisLook));
		glm::quat paddleQuat = glm::angleAxis(theta, axis);

		btQuaternion paddleQuat1 = GLToBtQuat(paddleQuat);

		newTrans.setRotation(paddleQuat1*newTrans.getRotation());
		BlockCounter1[15]->rigidBody->getMotionState()->setWorldTransform(newTrans);
		newTrans.setIdentity();
	}
	else if(gameStarted && movingLeft)
	{
		movingLeft = false;
	}

	if(BlockCounter1[15]->position.z > 70 && gameStarted && movingLeft==false)
	{
		btTransform newTrans;
		
		BlockCounter1[15]->rigidBody->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin() += btVector3(0,0,-1);
		

		glm::vec3 axis = glm::cross(GameComponent::basisRight, GameComponent::basisLook);
		axis = glm::normalize(axis);
		float theta = -5.0f;//glm::acos(glm::dot(GameComponent::basisUp, GameComponent::basisLook));
		glm::quat paddleQuat = glm::angleAxis(theta, axis);

		btQuaternion paddleQuat1 = GLToBtQuat(paddleQuat);

		newTrans.setRotation(paddleQuat1*newTrans.getRotation());
		BlockCounter1[15]->rigidBody->getMotionState()->setWorldTransform(newTrans);
		newTrans.setIdentity();
	}
	else if(gameStarted && movingLeft==false)
	{
		movingLeft = true;
	}
}

void BGE::Breakout2D::Cleanup()
{
	Game::Cleanup();
}

void BGE::Breakout2D::UpdateCollisions()
{
	// Collision checks to move any hit walls
	//ideally this would be able to tell which wall it is by itself
	//but again shortcuts had to be taken so i must call each wall by its tag
    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* obA = (btCollisionObject*)(contactManifold->getBody0());
        btCollisionObject* obB = (btCollisionObject*)(contactManifold->getBody1());
        PhysicsController * pcA = reinterpret_cast<PhysicsController*>(obA->getUserPointer());
        PhysicsController * pcB = reinterpret_cast<PhysicsController*>(obB->getUserPointer());

        int numContacts = contactManifold->getNumContacts();
        if (numContacts > 0)
        {
            if ((pcA != nullptr) && (pcB != nullptr))
            {
                            

				if(pcB->rigidBody->isKinematicObject())
				{
					//PrintText("Collision between " + pcA->tag + " and " + pcB->tag);

					if(pcB->tag=="deleteMe")
					{
						//ideally this item would delete but could figure out how to do this 
						//BlockCounter->Cleanup();
						//dynamicsWorld->removeCollisionObject(obB);
						//delete obB;

						btTransform newTrans;

						BlockCounter1[0]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[0]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe1")
					{
						btTransform newTrans;

						BlockCounter1[1]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[1]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe2")
					{
						btTransform newTrans;

						BlockCounter1[2]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[2]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe3")
					{
						btTransform newTrans;

						BlockCounter1[3]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[3]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe4")
					{
						btTransform newTrans;

						BlockCounter1[4]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[4]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe4")
					{
						btTransform newTrans;

						BlockCounter1[4]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[4]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe5")
					{
						btTransform newTrans;

						BlockCounter1[5]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[5]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe6")
					{
						btTransform newTrans;

						BlockCounter1[6]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[6]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe7")
					{
						btTransform newTrans;

						BlockCounter1[7]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[7]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe8")
					{
						btTransform newTrans;

						BlockCounter1[8]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[8]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe9")
					{
						btTransform newTrans;

						BlockCounter1[9]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[9]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe10")
					{
						btTransform newTrans;

						BlockCounter1[10]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[10]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
						ballSpeed = 150;
					}
					else if(pcB->tag=="deleteMe11")
					{
						btTransform newTrans;

						BlockCounter1[11]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[11]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe12")
					{
						btTransform newTrans;

						BlockCounter1[12]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[12]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
					else if(pcB->tag=="deleteMe13")
					{
						btTransform newTrans;

						BlockCounter1[13]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[13]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
						ballSpeed = 150;
					}
					else if(pcB->tag=="deleteMe14")
					{
						btTransform newTrans;

						BlockCounter1[14]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[14]->rigidBody->getMotionState()->setWorldTransform(newTrans);
						blockCount--;
						soundSystem->PlaySound("Hit1_01", glm::vec3(0,0,0));
					}
				}
            }
        }
    }
}

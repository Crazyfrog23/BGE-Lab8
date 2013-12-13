#include "PhysicsGame1.h"
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

using namespace BGE;

PhysicsGame1::PhysicsGame1(void)
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
}

PhysicsGame1::~PhysicsGame1(void)
{
}


std::shared_ptr<GameComponent> station;

bool PhysicsGame1::Initialise() 
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
	
	// Now some constraints
	//shared_ptr<PhysicsController> box1 = physicsFactory->CreateBox(1,1,4, glm::vec3(5, 5, 0), glm::quat()); 
	//shared_ptr<PhysicsController> box2 = physicsFactory->CreateBox(1,1,4, glm::vec3(5, 5, 5), glm::quat());
	

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

	// A hinge
	//btHingeConstraint * hinge = new btHingeConstraint(*box1->rigidBody, *box2->rigidBody, btVector3(0,0,2.5f),btVector3(0,0,-2.5f), btVector3(0,1,0), btVector3(0,1,0), true);
	//dynamicsWorld->addConstraint(hinge);

	//box1 = physicsFactory->CreateBox(1,1,4, glm::vec3(10, 5, 0), glm::quat()); 
	//box2 = physicsFactory->CreateBox(1,1,4, glm::vec3(10, 5, 5), glm::quat());

	//physicsFactory->CreateCylinder(10, 3, glm::vec3(0, 20, 0), glm::quat());

	std::shared_ptr<GameComponent> ship = make_shared<GameComponent>();
	ship->ambient = glm::vec3(0.2f, 0.2, 0.2f);
	ship->specular = glm::vec3(1.2f, 1.2f, 1.2f);
	std::shared_ptr<Model> model = Content::LoadModel("cobramk3", glm::rotate(glm::mat4(1), 180.0f, GameComponent::basisUp));	
	std::shared_ptr<GameComponent> steerable = make_shared<Steerable3DController>(model);
	steerable->position = glm::vec3(20, 5, -20);
	std::shared_ptr<VectorDrawer> vectorDrawer = make_shared<VectorDrawer>();
	vectorDrawer->scale = glm::vec3(5,5,10);
	ship->Attach(steerable);
	ship->Attach(model);
	ship->Attach(vectorDrawer);
	Attach(ship);

	// Create a hierarchy
	station = make_shared<GameComponent>();
	station->worldMode = world_modes::from_self;
	station->ambient = glm::vec3(0.2f, 0.2, 0.2f);
	station->specular = glm::vec3(0,0,0);
	station->scale = glm::vec3(2,2,2);
	std::shared_ptr<Model> cmodel = Content::LoadModel("coriolis", glm::rotate(glm::mat4(1), 90.0f, GameComponent::basisUp));	
	station->Attach(cmodel);
	station->Attach(make_shared<VectorDrawer>(glm::vec3(7,7,7)));
	station->position = glm::vec3(40, 5, -20);
	Attach(station);

	// Add a child to the station and update by including the parent's world transform
	std::shared_ptr<GameComponent> ship1 = make_shared<GameComponent>();
	ship1->worldMode = world_modes::from_self_with_parent;
	ship1->ambient = glm::vec3(0.2f, 0.2, 0.2f);
	ship1->specular = glm::vec3(1.2f, 1.2f, 1.2f);
	std::shared_ptr<Model> ana = Content::LoadModel("anaconda", glm::rotate(glm::mat4(1), 180.0f, GameComponent::basisUp));	
	ship1->Attach(ana);
	ship1->position = glm::vec3(0, 0, -10);
	station->Attach(ship1);

	//physicsFactory->CreateVehicle(glm::vec3(0,10,-30));
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

void BGE::PhysicsGame1::Update(float timeDelta)
{
	dynamicsWorld->stepSimulation(timeDelta,7);
	station->Yaw(timeDelta * 20.0f);
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
	}

	//start new game
	if(keyState[SDL_SCANCODE_F4])
	{
		Ball = physicsFactory->CreateSphere(3, glm::vec3(100+50, 1, 100), glm::quat());
		Ball->rigidBody->setLinearVelocity(GLToBtVector(glm::vec3(-120,0,rand()%(40+40 + 1) - 40)));   
		Ball->rigidBody->activate();
	}

	//update collsions of objects
	UpdateCollisions();
	
}

void BGE::PhysicsGame1::Cleanup()
{
	Game::Cleanup();
}

void BGE::PhysicsGame1::UpdateCollisions()
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

					}
					else if(pcB->tag=="deleteMe1")
					{
						btTransform newTrans;

						BlockCounter1[1]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[1]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe2")
					{
						btTransform newTrans;

						BlockCounter1[2]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[2]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe3")
					{
						btTransform newTrans;

						BlockCounter1[3]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[3]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe4")
					{
						btTransform newTrans;

						BlockCounter1[4]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[4]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe4")
					{
						btTransform newTrans;

						BlockCounter1[4]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[4]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe5")
					{
						btTransform newTrans;

						BlockCounter1[5]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[5]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe6")
					{
						btTransform newTrans;

						BlockCounter1[6]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[6]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe7")
					{
						btTransform newTrans;

						BlockCounter1[7]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[7]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe8")
					{
						btTransform newTrans;

						BlockCounter1[8]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[8]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe9")
					{
						btTransform newTrans;

						BlockCounter1[9]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[9]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe10")
					{
						btTransform newTrans;

						BlockCounter1[10]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[10]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe11")
					{
						btTransform newTrans;

						BlockCounter1[11]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[11]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe12")
					{
						btTransform newTrans;

						BlockCounter1[12]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[12]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe13")
					{
						btTransform newTrans;

						BlockCounter1[13]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[13]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
					else if(pcB->tag=="deleteMe14")
					{
						btTransform newTrans;

						BlockCounter1[14]->rigidBody->getMotionState()->getWorldTransform(newTrans);
						newTrans.getOrigin() += btVector3(0,0,-200);
						BlockCounter1[14]->rigidBody->getMotionState()->setWorldTransform(newTrans);
					}
				}
            }
        }
    }
}

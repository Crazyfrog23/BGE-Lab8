#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include "EoinsAdditions.h"
#include <btBulletDynamicsCommon.h>
#include "Box.h"
#include "FountainEffect.h"

namespace BGE
{
	class PhysicsGame1 :
		public Game
	{
	private:
		btBroadphaseInterface* broadphase;
 
		// Set up the collision configuration and dispatcher
		btDefaultCollisionConfiguration * collisionConfiguration;
		btCollisionDispatcher * dispatcher;
 
		// The actual physics solver
		btSequentialImpulseConstraintSolver * solver;

		int paddleSpeed;

	public:
		PhysicsGame1(void);
		~PhysicsGame1(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void UpdateCollisions();
		
		// The world.
		std::shared_ptr<PhysicsFactory> physicsFactory;
		std::shared_ptr<EoinsAdditions> eoinsFactory;
		btDiscreteDynamicsWorld * dynamicsWorld;
		shared_ptr<PhysicsController> breakoutPaddle;
		shared_ptr<PhysicsController> Ball;
		vector<shared_ptr<PhysicsController>> BlockCounter1;
		shared_ptr<PhysicsController> BlockCounter;
		glm:: vec3 paddlePosition;

		int ballSpeed;
		shared_ptr<FountainEffect> Fountain0;
	};
}

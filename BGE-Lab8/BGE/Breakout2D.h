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
	class Breakout2D :
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
		Breakout2D(void);
		~Breakout2D(void);
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
		glm:: vec3 paddlePosition;

		int ballSpeed;
		int blockCount;
		string currentHighScoreString;
		int currentHighScore;
		int CountDownTimer;
		bool gameStarted;
		bool movingLeft;
		shared_ptr<FountainEffect> Fountain0;
	};
}

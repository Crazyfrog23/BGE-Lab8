#pragma once
#include <memory>
#include "PhysicsController.h"
#include "GameComponent.h"
#include "PhysicsFactory.h"
#include "Game.h"
#include "Box.h"

using namespace std;

namespace BGE
{
	class EoinsAdditions
	{
	private:
		
	public:

		EoinsAdditions();
		~EoinsAdditions(void);

		shared_ptr<PhysicsController> NewCreateBox(float width, float height, float depth, glm::vec3 pos, glm::quat quat, int massIn, float margin, int friction, int restitution, bool kinimatic,btDiscreteDynamicsWorld * dynamicsWorld, string tagIn);
		void CreateWall(glm::vec3 startAt, float width, float height, btDiscreteDynamicsWorld * dynamicsWorld, float blockWidth = 5, float blockHeight = 5, float blockDepth = 5);
		void CreatePyramid(float width, float height, float depth, glm::vec3 centre, glm::quat quat, int towerHeight, int towerWidth,btDiscreteDynamicsWorld * dynamicsWorld);
		void CreateRagDoll(glm::vec3 scale, glm::vec3 pos, glm::quat quat, shared_ptr<PhysicsFactory> phyFactory,btDiscreteDynamicsWorld * dynamicsWorld);
		void CreateSnookerTable(float width, float height, glm::vec3 centre, glm::quat quat, shared_ptr<PhysicsFactory> phyFactory,btDiscreteDynamicsWorld * dynamicsWorld);
		shared_ptr<PhysicsController> CreateBreakout(float width, float height, glm::vec3 centre, glm::quat quat, shared_ptr<PhysicsFactory> phyFactory,btDiscreteDynamicsWorld * dynamicsWorld);
		void TakeScreenShot();

		
	};
}

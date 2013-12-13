#pragma once
#include <memory>
#include "PhysicsController.h"
#include "GameComponent.h"

using namespace std;

namespace BGE
{
	class PhysicsFactory
	{
	private:
		
	public:
		PhysicsFactory(btDiscreteDynamicsWorld * dynamicsWorld);
		~PhysicsFactory(void);

		btCollisionShape * boxShape;

		shared_ptr<PhysicsController> CreateBox(float width, float height, float depth, glm::vec3 pos, glm::quat quat);
		shared_ptr<PhysicsController> CreateSphere(float radius, glm::vec3 pos, glm::quat quat);
		shared_ptr<PhysicsController> CreateCylinder(float radius, float height, glm::vec3 pos, glm::quat quat);
		shared_ptr<PhysicsController> CreateVehicle(glm::vec3 pos);
		shared_ptr<PhysicsController> CreateCameraPhysics();
		shared_ptr<PhysicsController> CreateGroundPhysics();
		shared_ptr<PhysicsController> CreateFromModel(string name, glm::vec3 pos, glm::quat quat, glm::vec3 scale = glm::vec3(1));
		
		shared_ptr<PhysicsController> CreateRandomObject(glm::vec3 point, glm::quat q);
		btDiscreteDynamicsWorld * dynamicsWorld;

		shared_ptr<PhysicsController> NewCreateBox(float width, float height, float depth, glm::vec3 pos, glm::quat quat, int massIn, bool kinimatic);
		void CreateWall(glm::vec3 startAt, float width, float height, float blockWidth = 5, float blockHeight = 5, float blockDepth = 5);
		void CreatePyramid(float width, float height, float depth, glm::vec3 centre, glm::quat quat, int towerHeight, int towerWidth);
		void CreateRagDoll(glm::vec3 scale, glm::vec3 pos, glm::quat quat);
		void CreateSnookerTable(float width, float height, glm::vec3 centre, glm::quat quat);
		void CreateText();
	};
}

#ifndef COLLIDER_H_
#define COLLIDER_H_

#include "glm/glm.hpp"
#include <vector>

struct Collision {
	glm::vec3 normal;
	glm::vec3 collisionPoint;
};

class Collider {
public:
	virtual ~Collider() {}
	virtual bool checkCollision(glm::vec3 point, Collision& collision) = 0;
};

class SphereCollider : public Collider {
public:
	SphereCollider(glm::vec3 center, float radius) : center(center), radius(radius) {}
	virtual ~SphereCollider() {}
	bool checkCollision(glm::vec3 point, Collision& collision);

	glm::vec3 center;
private:
	float radius;
};

class SimpleClothCollider : public Collider {
public:
	SimpleClothCollider(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& normals) : positions(positions), normals(normals) {}
	virtual ~SimpleClothCollider() {}
	bool checkCollision(glm::vec3 point, Collision& collision);

private:
	const std::vector<glm::vec3>& positions;
	const std::vector<glm::vec3>& normals;
};

#endif
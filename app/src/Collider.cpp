#include "Collider.h"

#include <iostream>
bool SphereCollider::checkCollision(glm::vec3 point, Collision& collision) {
	float distance = glm::length(point - center);
	//std::cout << point[0] << "," << point[1] << "," << point[2] << " " << distance << "" << radius << std::endl;
	if (distance < radius + 0.0001f) {
		collision.normal = glm::normalize(point - center);
		collision.collisionPoint = point + collision.normal*(0.001f + radius - distance);
		return true;
	}

	return false;
}
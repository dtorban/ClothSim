#include "Collider.h"

bool SphereCollider::checkCollision(glm::vec3 point, Collision& collision) {
	float distance = glm::length(point - center);
	if (distance < radius + 0.0001f) {
		collision.normal = glm::normalize(point - center);
		collision.collisionPoint = point + collision.normal*(0.001f + radius - distance);
		return true;
	}

	return false;
}

bool SimpleClothCollider::checkCollision(glm::vec3 point, Collision& collision) {
	/*float distance = glm::length(point - center);
	if (distance < radius + 0.0001f) {
		collision.normal = glm::normalize(point - center);
		collision.collisionPoint = point + collision.normal*(0.001f + radius - distance);
		return true;
	}*/

	return false;
}
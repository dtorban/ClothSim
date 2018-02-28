#include "MassSpringSystem.h"

MassSpringSystem::~MassSpringSystem() {
	for (int f = 0; f < forces.size(); f++) {
		delete forces[f];
	}

	for (int f = 0; f < colliders.size(); f++) {
		delete colliders[f];
	}
}

int MassSpringSystem::getDOFs() const {
	return positions.size()*3;
}

void MassSpringSystem::getState(VectorXd &x, VectorXd &v) const {
	for (int f = 0; f < positions.size(); f++) {
		x[f*3] = positions[f][0];
		x[f*3+1] = positions[f][1];
		x[f*3+2] = positions[f][2];
		v[f*3] = velocities[f][0];
		v[f*3+1] = velocities[f][1];
		v[f*3+2] = velocities[f][2];
	}
}

void MassSpringSystem::setState(const VectorXd &x, const VectorXd &v) {
	for (int f = 0; f < positions.size(); f++) {
		positions[f][0] = x[f*3];
		positions[f][1] = x[f*3+1];
		positions[f][2] = x[f*3+2];
		velocities[f][0] = v[f*3];
		velocities[f][1] = v[f*3+1];
		velocities[f][2] = v[f*3+2];
		points[f][0] = x[f*3];
		points[f][1] = x[f*3+1];
		points[f][2] = x[f*3+2];
	}
}

void MassSpringSystem::getInertia(MatrixXd &M) const {
	M.setZero();
	for (int f = 0; f < masses.size(); f++) {
		M.block(f*3,f*3, 3,3) = masses[f]*MatrixXd::Identity(3,3);
	}
}

void MassSpringSystem::getForces(const VectorXd &x, const VectorXd &v, VectorXd &f) const {
	f.setZero();
	for (int i = 0; i < forces.size(); i++) {
		forces[i]->addForces(x, v, f);
	}
}

void MassSpringSystem::getAcceleration(const VectorXd &x, const VectorXd &v, VectorXd &a) const {
	a.setZero();
	for (int i = 0; i < forces.size(); i++) {
		forces[i]->addAcceleration(x, v, a);
	}
}

void MassSpringSystem::addNode(double mass, glm::vec3 position) {
	masses.push_back(mass);
	positions.push_back(position);
	points.push_back(position);
	velocities.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
}

void MassSpringSystem::addForce(Force* force) {
	forces.push_back(force);
}

void MassSpringSystem::addCollider(Collider* collider) {
	colliders.push_back(collider);
}

void MassSpringSystem::handleCollisions() {
	Collision collision;
	for (int f = 0; f < positions.size(); f++) {
		for (int i = 0; i < colliders.size(); i++) {
			if(colliders[i]->checkCollision(points[f], collision)) {
				//std::cout << "Collision" << std::endl;
				glm::vec3 vel = velocities[f];
				glm::vec3 bounce = glm::dot(vel, collision.normal)*collision.normal;
				velocities[f] -= 0.5f*bounce;
				positions[f] = collision.collisionPoint;
			}
		}
	}
}

void MassSpringSystem::getJacobians(const VectorXd &x, const VectorXd &v, MatrixXd& jx, MatrixXd& jv) const {
	jx.setZero();
	jv.setZero();
	for (int i = 0; i < forces.size(); i++) {
		forces[i]->addJacobians(x, v, jx, jv);
	}
}
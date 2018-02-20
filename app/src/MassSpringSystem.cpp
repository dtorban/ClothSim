#include "MassSpringSystem.h"

MassSpringSystem::~MassSpringSystem() {
	for (int f = 0; f < forces.size(); f++) {
		delete forces[f];
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
	}
}

void MassSpringSystem::getInertia(MatrixXd &M) const {
	for (int f = 0; f < masses.size(); f++) {
		M(f,f) = masses[f];
	}
}

void MassSpringSystem::getForces(VectorXd &f) const {
}

void MassSpringSystem::addNode(double mass, glm::vec3& position) {
	masses.push_back(mass);
	positions.push_back(position);
	velocities.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
}

void MassSpringSystem::addForce(Force* force) {
	forces.push_back(force);
}

#ifndef FORCE_H_
#define FORCE_H_

#include "glm/glm.hpp"

#include <Eigen/Dense>
using namespace Eigen;

class Force {
public:
	virtual ~Force() {}
	virtual void addForces(VectorXd &f) = 0;
};

class ConstantForce : public Force {
public:
	ConstantForce(glm::vec3 vec, int numNodes, int positionOffset) : vec(vec), numNodes(numNodes), positionOffset(positionOffset) {}
	void addForces(VectorXd &f);
private:
	glm::vec3 vec;
	int numNodes;
	int positionOffset;
};

#endif
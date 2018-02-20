#ifndef FORCE_H_
#define FORCE_H_

#include "glm/glm.hpp"

#include <Eigen/Dense>
using namespace Eigen;

class Force {
public:
	virtual ~Force() {}
	virtual void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) = 0;
};

class ConstantForce : public Force {
public:
	ConstantForce(glm::vec3 vec, int numNodes, int positionOffset) : vec(vec), numNodes(numNodes), positionOffset(positionOffset) {}
	void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f);
private:
	glm::vec3 vec;
	int numNodes;
	int positionOffset;
};

class AnchorForce : public Force {
public:
	AnchorForce(int node, glm::vec3 anchorPoint, double ks, double kd, int numNodes, int positionOffset) : node(node), anchorPoint(anchorPoint), numNodes(numNodes), positionOffset(positionOffset), ks(ks), kd(kd) {}
	void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f);
private:
	int node;
	glm::vec3 anchorPoint;
	int numNodes;
	int positionOffset;
	double ks;
	double kd;
};


#endif
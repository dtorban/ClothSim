#ifndef FORCE_H_
#define FORCE_H_

#include "glm/glm.hpp"

#include <Eigen/Dense>
using namespace Eigen;

class Force {
public:
	virtual ~Force() {}
	virtual void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {}
	virtual void addAcceleration(const VectorXd &x, const VectorXd &v, VectorXd &a) {}
	virtual void addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv) {}
};

class ConstantForce : public Force {
public:
	ConstantForce(glm::vec3 vec, int numNodes, int positionOffset) : vec(vec), numNodes(numNodes), positionOffset(positionOffset) {}
	void addAcceleration(const VectorXd &x, const VectorXd &v, VectorXd &a);
	void addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv);
private:
	glm::vec3 vec;
	int numNodes;
	int positionOffset;
};

class AnchorForce : public Force {
public:
	AnchorForce(int node, glm::vec3 anchorPoint, double& ks, double& kd, int numNodes, int positionOffset) : node(node), anchorPoint(anchorPoint), numNodes(numNodes), positionOffset(positionOffset), ks(ks), kd(kd) {}
	void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f);
	void addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv);
private:
	int node;
	glm::vec3 anchorPoint;
	int numNodes;
	int positionOffset;
	double& ks;
	double& kd;
};

class SpringForce : public Force {
public:
	SpringForce(int node1, int node2, double& ks, double& kd, double l0, int numNodes, int positionOffset, bool record = false)
	 : node1(node1), node2(node2), numNodes(numNodes), positionOffset(positionOffset), ks(ks), kd(kd), l0(l0), record(record) {}
	void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f);
	void addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv);
private:
	int node1;
	int node2;
	int numNodes;
	int positionOffset;
	double& ks;
	double& kd;
	double l0;
	bool record;
};

class AreoForce : public Force {
public:
	AreoForce(int node1, int node2, int node3, double density, double cd, glm::vec3 airV, int numNodes, int positionOffset)
	 : density(density), cd(cd), airV(airV), numNodes(numNodes), positionOffset(positionOffset) {
		nodes[0] = node1;
		nodes[1] = node2;
		nodes[2] = node3;
	}
	void addForces(const VectorXd &x, const VectorXd &v, VectorXd &f);
private:
	int nodes[3];
	double density;
	double cd;
	int numNodes;
	int positionOffset;
	glm::vec3 airV;
};

#endif
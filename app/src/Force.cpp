#include "Force.h"

#include <iostream>

void ConstantForce::addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {
	int nodeSize = f.size()/numNodes;
	for (int i = 0; i < numNodes; i++) {
		f[i*nodeSize + positionOffset + 0] += vec[0];
		f[i*nodeSize + positionOffset + 1] += vec[1];
		f[i*nodeSize + positionOffset + 2] += vec[2];
	}
}

void AnchorForce::addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {
	int nodeSize = f.size()/numNodes;
	Vector3d dx = x.segment(nodeSize*node + positionOffset, 3) - Vector3d(anchorPoint[0], anchorPoint[1], anchorPoint[2]);
	f.segment(nodeSize*node + positionOffset, 3) += -ks*dx - kd*v.segment(nodeSize*node + positionOffset, 3);
}

void SpringForce::addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {
	int nodeSize = f.size()/numNodes;
  	Vector3d l = x.segment(nodeSize*node1 + positionOffset, 3) - x.segment(nodeSize*node2 + positionOffset, 3);
	Vector3d dl = v.segment(nodeSize*node1 + positionOffset, 3) - v.segment(nodeSize*node2 + positionOffset, 3);
	f.segment(nodeSize*node1 + positionOffset, 3) += -(ks*(l.norm() - l0) + kd*dl.dot(l)/l.norm())*l/l.norm();
	f.segment(nodeSize*node2 + positionOffset, 3) += (ks*(l.norm() - l0) + kd*dl.dot(l)/l.norm())*l/l.norm();
}
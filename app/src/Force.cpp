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
	Vector3d calc = f.segment(nodeSize*node + positionOffset, 3);
	std::cout <<"a:" << node << "," << calc[0] << "," << calc[1] << "," << calc[2] << std::endl;
	calc = -ks*dx - kd*v.segment(nodeSize*node + positionOffset, 3);
	f.segment(nodeSize*node + positionOffset, 3) += calc;
	calc = f.segment(nodeSize*node + positionOffset, 3);
	std::cout <<"b:" << node << "," << calc[0] << "," << calc[1] << "," << calc[2] << std::endl;
}
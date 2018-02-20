#include "Force.h"

#include <iostream>

void ConstantForce::addForces(VectorXd &f) {
	int nodeSize = f.size()/numNodes;
	for (int i = 0; i < numNodes; i++) {
		f[i*nodeSize + positionOffset + 0] = vec[0];
		f[i*nodeSize + positionOffset + 1] = vec[1];
		f[i*nodeSize + positionOffset + 2] = vec[2];
	}
}
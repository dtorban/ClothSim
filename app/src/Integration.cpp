#include "Integration.h"

void ExplicitIntegrator::step(PhysicalSystem& system, double dt) {
    int n = system.getDOFs();
    VectorXd x0(n), v0(n);
    system.getState(x0, v0);
    MatrixXd M(n,n);
    system.getInertia(M);
    VectorXd f0(n);
    system.getForces(x0, v0, f0);
    VectorXd a0(n); // acceleration
    for (int i = 0; i < n; i++) {
        a0(i) = f0(i)/M(i,i);
    }
    VectorXd x1 = x0 + v0*dt;
    VectorXd v1 = v0 + a0*dt;
	system.setState(x1, v1);
}
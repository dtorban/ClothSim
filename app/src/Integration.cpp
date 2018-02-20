#include "Integration.h"

struct ExplicitEulerMemory {
    ExplicitEulerMemory(int n) :
        x0(n), v0(n), f0(n), a0(n), x1(n), v1(n), M(n,n) {}
    VectorXd x0, v0, f0, a0, x1, v1;
    MatrixXd M;
};

void ExplicitEulerIntegrator::step(PhysicalSystem& system, double dt, void* memory) {
    ExplicitEulerMemory& mem = *static_cast<ExplicitEulerMemory*>(memory);
    int n = system.getDOFs();
    system.getState(mem.x0, mem.v0);
    system.getInertia(mem.M);
    system.getForces(mem.x0, mem.v0, mem.f0);
    for (int i = 0; i < n; i++) {
        mem.a0(i) = mem.f0(i)/mem.M(i,i);
    }
    mem.x1 = mem.x0 + mem.v0*dt;
    mem.v1 = mem.v0 + mem.a0*dt;
	system.setState(mem.x1, mem.v1);
}

void* ExplicitEulerIntegrator::allocateMemory(PhysicalSystem& system) {
    return new ExplicitEulerMemory(system.getDOFs());
}

void ExplicitEulerIntegrator::freeMemory(void* memory) {
    delete static_cast<ExplicitEulerMemory*>(memory);
}

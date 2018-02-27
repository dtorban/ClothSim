#include "Integration.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

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
    system.getAcceleration(mem.x0, mem.v0, mem.a0);
    system.getForces(mem.x0, mem.v0, mem.f0);
    for (int i = 0; i < n; i++) {
        mem.a0(i) += mem.f0(i)/mem.M(i,i);
    }

    if (semiImplicit) {
        mem.v1 = mem.v0 + mem.a0*dt;
        mem.x1 = mem.x0 + mem.v1*dt;   
    }
    else {
        mem.x1 = mem.x0 + mem.v0*dt;
        mem.v1 = mem.v0 + mem.a0*dt;   
    }

	system.setState(mem.x1, mem.v1);
}

void* ExplicitEulerIntegrator::allocateMemory(PhysicalSystem& system) {
    return new ExplicitEulerMemory(system.getDOFs());
}

void ExplicitEulerIntegrator::freeMemory(void* memory) {
    delete static_cast<ExplicitEulerMemory*>(memory);
}



struct RungaKutta4Memory {
    RungaKutta4Memory(int n) :
        x0(n), v0(n), f0(n), a0(n), x1(n), v1(n), M(n,n), 
        k1X(n), k2X(n), k3X(n), k4X(n), 
        k1V(n), k2V(n), k3V(n), k4V(n), 
        tempX(n), tempV(n), n(n) {}
    VectorXd x0, v0, f0, a0, x1, v1, tempX, tempV;
    VectorXd k1X, k2X, k3X, k4X;
    VectorXd k1V, k2V, k3V, k4V;
    MatrixXd M;
    int n;
};

void RungaKutta4_calc_deriviative(PhysicalSystem& system, RungaKutta4Memory& mem, double dt, 
    const VectorXd& x, const VectorXd& v, VectorXd& derivativeX, VectorXd derivativeV, VectorXd& derivativeOutX, VectorXd derivativeOutV, VectorXd& tempX, VectorXd& tempV) {

    tempX = x + derivativeX*dt;
    tempV = v + derivativeV*dt;

    system.getAcceleration(tempX, tempV, mem.a0);
    system.getForces(tempX, tempV, mem.f0);
    for (int i = 0; i < mem.n; i++) {
        mem.a0(i) += mem.f0(i)/mem.M(i,i);
    }

    derivativeOutV = mem.a0;
    derivativeOutX = v + mem.a0*dt;
}

void RungaKutta4Integrator::step(PhysicalSystem& system, double dt, void* memory) {

    RungaKutta4Memory& mem = *static_cast<RungaKutta4Memory*>(memory);
    int n = system.getDOFs();
    system.getState(mem.x0, mem.v0);
    system.getInertia(mem.M);

    RungaKutta4_calc_deriviative(system, mem, 0.0, mem.x0, mem.v0, mem.k1X, mem.k1V, mem.k1X, mem.k1V, mem.tempX, mem.tempV);
    RungaKutta4_calc_deriviative(system, mem, dt/2.0, mem.x0, mem.v0, mem.k1X, mem.k1V, mem.k2X, mem.k2V, mem.tempX, mem.tempV);
    RungaKutta4_calc_deriviative(system, mem, dt/2.0, mem.x0, mem.v0, mem.k2X, mem.k2V, mem.k3X, mem.k3V, mem.tempX, mem.tempV);
    RungaKutta4_calc_deriviative(system, mem, dt, mem.x0, mem.v0, mem.k3X, mem.k3V, mem.k4X, mem.k4V, mem.tempX, mem.tempV);

    mem.v0 = 1.0 / 6.0 * ( mem.k1X + 2.0*(mem.k2X + mem.k3X) + mem.k4X );
    mem.a0 = 1.0 / 6.0 * ( mem.k1V + 2.0*(mem.k2V + mem.k3V) + mem.k4V );

    mem.x1 = mem.x0 + mem.v0*dt;
    mem.v1 = mem.v0 + mem.a0*dt;

    system.setState(mem.x1, mem.v1);
}

void* RungaKutta4Integrator::allocateMemory(PhysicalSystem& system) {
    return new RungaKutta4Memory(system.getDOFs());
}

void RungaKutta4Integrator::freeMemory(void* memory) {
    delete static_cast<RungaKutta4Memory*>(memory);
}



VectorXd solve(const MatrixXd &A, const VectorXd &b) {
    SparseMatrix<double> spA = A.sparseView();
    ConjugateGradient< SparseMatrix<double> > solver;
    return solver.compute(spA).solve(b);
}

struct ImplicitEulerMemory {
    ImplicitEulerMemory(int n) :
        x0(n), v0(n), f0(n), a0(n), x1(n), v1(n), dv(n), b(n), M(n,n), jx(n,n), jv(n,n), A(n,n) {}
    VectorXd x0, v0, f0, a0, x1, v1, dv, b;
    MatrixXd M, jx, jv, A;
};

#include <iostream>

void ImplicitEulerIntegrator::step(PhysicalSystem& system, double dt, void* memory) {
    ImplicitEulerMemory& mem = *static_cast<ImplicitEulerMemory*>(memory);
    int n = system.getDOFs();
    system.getState(mem.x0, mem.v0);
    system.getInertia(mem.M);
    system.getForces(mem.x0, mem.v0, mem.f0);
    system.getAcceleration(mem.x0, mem.v0, mem.a0);
    for (int i = 0; i < n; i++) {
        mem.f0(i) += mem.a0(i)*mem.M(i,i);
    }

    system.getJacobians(mem.x0, mem.v0, mem.jx, mem.jv);

    mem.A = mem.M - mem.jx*(dt*dt) - mem.jv*dt;
    mem.b = (mem.f0 + mem.jx*mem.v0*dt)*dt;

    /*for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                std::cout << mem.A(3*50+i,3*50+j) << " ";
            }
            std::cout  << std::endl;
        }
        std::cout << std::endl;*/

    mem.dv = solve(mem.A,mem.b);

    mem.v1 = mem.v0 + mem.dv;
    //std::cout << "before: " << mem.v1[0] << " " << mem.v1[1] << " " << mem.v1[2] << std::endl;
    //mem.v1 += mem.a0*dt;
    //std::cout << "after: " << mem.v1[0] << " " << mem.v1[1] << " " << mem.v1[2] << std::endl;

    mem.x1 = mem.x0 + mem.v1*dt;

    std::cout << "before: " << mem.x0[0] << " " << mem.x0[1] << " " << mem.x0[2] << std::endl;
    std::cout << "after: " << mem.x1[0] << " " << mem.x1[1] << " " << mem.x1[2] << std::endl;

    system.setState(mem.x1, mem.v1);

    /*
    int n = system->getDOFs();
    VectorXd x0(n), v0(n);
    system->getState(x0, v0);
    MatrixXd M(n,n);
    system->getInertia(M);
    VectorXd f0(n);
    system->getForces(f0);
    MatrixXd jx(n,n), jv(n,n);
    system->getJacobians(jx, jv);
    MatrixXd A(n,n);
    A = M - jx*(dt*dt) - jv*dt;
    VectorXd b(n);
    b = (f0 + jx*v0*dt)*dt;
    VectorXd dv = solve(A,b);
    VectorXd v1 = v0 + dv;
    VectorXd x1 = x0 + v1*dt;
    system->setState(x1, v1);
    */
}

void* ImplicitEulerIntegrator::allocateMemory(PhysicalSystem& system) {
    return new ImplicitEulerMemory(system.getDOFs());
}

void ImplicitEulerIntegrator::freeMemory(void* memory) {
    delete static_cast<ImplicitEulerMemory*>(memory);
}
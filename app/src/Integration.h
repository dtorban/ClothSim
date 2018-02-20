#ifndef INTEGRATION_H_
#define INTEGRATION_H_

#include <Eigen/Dense>
using namespace Eigen;

class PhysicalSystem {
public:
	virtual ~PhysicalSystem() {}
    virtual int getDOFs() const = 0;
    virtual void getState(VectorXd &x, VectorXd &v) const = 0;
    virtual void setState(const VectorXd &x, const VectorXd &v) = 0;
    virtual void getInertia(MatrixXd &M) const = 0;
    virtual void getForces(const VectorXd &x, const VectorXd &v, VectorXd &f) const = 0;
    virtual void getAcceleration(const VectorXd &x, const VectorXd &v, VectorXd &a) const = 0;
};

class Integrator {
public:
	virtual ~Integrator() {}
	virtual void* allocateMemory(PhysicalSystem& system) = 0;
	virtual void freeMemory(void* memory) = 0;
	virtual void step(PhysicalSystem& system, double dt, void* memory) = 0;
};

class ExplicitEulerIntegrator : public Integrator {
public:
	ExplicitEulerIntegrator(bool semiImplicit = false) : semiImplicit(semiImplicit) {}	
	virtual ~ExplicitEulerIntegrator() {}	
	void* allocateMemory(PhysicalSystem& system);
	void freeMemory(void* memory);
	void step(PhysicalSystem& system, double dt, void* memory);

private:
	bool semiImplicit;
};

#endif
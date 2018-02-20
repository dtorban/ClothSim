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
};

class Integrator {
public:
	virtual ~Integrator() {}
	virtual void step(PhysicalSystem& system, double dt) = 0;
};

class ExplicitIntegrator : public Integrator {
public:
	virtual ~ExplicitIntegrator() {}
	void step(PhysicalSystem& system, double dt);
};

#endif
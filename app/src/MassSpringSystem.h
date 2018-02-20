#ifndef MASSSPRINGSYSTEM_H_
#define MASSSPRINGSYSTEM_H_

#include "Integration.h"
#include <vector>
#include "glm/glm.hpp"
#include "Force.h"

class MassSpringSystem : public PhysicalSystem {
public:
	virtual ~MassSpringSystem();
	int getDOFs() const;
    void getState(VectorXd &x, VectorXd &v) const;
    void setState(const VectorXd &x, const VectorXd &v);
    void getInertia(MatrixXd &M) const;
    void getForces(const VectorXd &x, const VectorXd &v, VectorXd &f) const;

    void addNode(double mass, glm::vec3 position);
    void addForce(Force* force);

    const std::vector<glm::vec3>& getPositions() const { return positions; }

private:
	std::vector<double> masses;
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> velocities;
	std::vector<Force*> forces;
};

#endif
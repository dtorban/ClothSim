#ifndef MASSSPRINGSYSTEM_H_
#define MASSSPRINGSYSTEM_H_

#include "Integration.h"
#include <vector>
#include "glm/glm.hpp"
#include "Force.h"
#include "Collider.h"

class MassSpringSystem : public PhysicalSystem {
public:
	virtual ~MassSpringSystem();
	int getDOFs() const;
    void getState(VectorXd &x, VectorXd &v) const;
    void setState(const VectorXd &x, const VectorXd &v);
    void getInertia(MatrixXd &M) const;
    void getForces(const VectorXd &x, const VectorXd &v, VectorXd &f) const;
    void getAcceleration(const VectorXd &x, const VectorXd &v, VectorXd &a) const;
    void getJacobians(const VectorXd &x, const VectorXd &v, MatrixXd& jx, MatrixXd& jv) const;

    void addNode(double mass, glm::vec3 position);
    void addForce(Force* force);
    void addCollider(Collider* collider);
    void handleCollisions();

    const std::vector<glm::vec3>& getPositions() const { return points; }

private:
	std::vector<double> masses;
	std::vector<glm::dvec3> positions;
	std::vector<glm::dvec3> velocities;
    std::vector<glm::vec3> points;
	std::vector<Force*> forces;
	std::vector<Collider*> colliders;
};

#endif
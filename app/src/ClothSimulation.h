#include "MassSpringSystem.h"
#include "Integration.h"
#include "glm/glm.hpp"
#include <vector>

class ClothSimulation : public MassSpringSystem {
public:
	ClothSimulation(Integrator& integrator);
	virtual ~ClothSimulation();

	void step(double dt);

private:
	std::vector<glm::vec3> calculateNormals(const std::vector<unsigned int>& inds, const std::vector<glm::vec3>& positions);

	Integrator& integrator;
	void* integratorMemory;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> normals;
};
#include "ClothSimulation.h"

class ClothRenderer {
public:
	ClothRenderer(ClothSimulation& cloth) :cloth(cloth) {}
	virtual ~ClothRenderer() {}

	void createSharedContext();
	void createRenderer();

private:
	ClothSimulation& cloth;
}
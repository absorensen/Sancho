#include "octree.h"

octree::octree() {
	children = NULL;
	coplanar = false;
	centroid = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
	color = glm::vec4(0.5f, 0.5f, 0.5f, 0.0f);
	variance1 = variance2 = variance3 = 0.0f;
	votes = 0;
}

void octree::clear() {
	if (children != NULL)
	{
		for (unsigned int i = 0; i < 8; ++i)
		{
			children[i].indeces.clear();
			children[i].clear();
		}
		delete[] children;
		children = NULL;
	}
	children = NULL;
}
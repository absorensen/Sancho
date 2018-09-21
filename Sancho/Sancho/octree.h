#pragma once

#ifndef OCTREE_H
#define OCTREE_H

#include "glm/glm.hpp"

class octree {
public:
	octree();
	
	void clear();


	octree * children;
	bool coplanar;
	float variance1, variance2, variance3;
	int votes;
	glm::vec4 centroid, color;


private:

};


#endif
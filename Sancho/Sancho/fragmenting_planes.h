// all files with the prefix fp_ or fragmenting_planes
// are based on 3DKHT: 
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

#pragma once
#ifndef FRAGMENTING_PLANES_H
#define FRAGMENTING_PLANES_H

#include "fp_settings.h"
#include "fp_octree.h"

class fragmenting_planes {
public:
	fragmenting_planes();

	void run();




private:
	// detection
	void detection();


	// compression
	void compression();


	// validation
	void validation();


};



#endif
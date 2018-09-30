// all files with the prefix 3dkht_
// are based on 3DKHT: 
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

#pragma once
#ifndef FRAGMENTING_PLANES_H
#define FRAGMENTING_PLANES_H

#include "kht3d_settings.h"
#include "kht3d_octree.h"

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
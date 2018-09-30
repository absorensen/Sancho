// TODO:
// port octrees
// fix the visualization of the octrees
// verify and visualize octrees
// port kernel estimation
// verify kernel estimation
// port voting
// verify voting
// port sorting accumulator cells
// verify sorting accumulator cells
// port peak detection
// verify peak detection

#include "fragmenting_planes.h"

fragmenting_planes::fragmenting_planes() {
	return;
}

void fragmenting_planes::run() {
	// detection
	detection();

	// compression
	compression();

	// validation
	validation();

	return;
}

// currently based around modified 3DKHT
void fragmenting_planes::detection() {
	// clustering
	// =======================
	
	// how to create and cluster an octree?


	// kernel estimation
	// =======================

	// voting
	// =======================


	// sorting accumulator cells
	// =======================

	// peak detection
	// =======================


}

void fragmenting_planes::compression() {

}

void fragmenting_planes::validation() {

}
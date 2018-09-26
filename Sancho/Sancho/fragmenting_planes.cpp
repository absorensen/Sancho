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


	// kernel estimation


	// voting


	// sorting accumulator cells


	// peak detection

}

void fragmenting_planes::compression() {

}

void fragmenting_planes::validation() {

}
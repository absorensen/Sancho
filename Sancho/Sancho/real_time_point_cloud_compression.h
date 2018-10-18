#pragma once

#include <iostream>
#include <fstream>

#include "PointCloud.h"
#include "EigenTree.h"
#include "Octree.h"
#include "settings.h"
#include "common_includes.h"

//Octree tree;
//Settings settings;

void real_time_point_cloud_compression(PointCloud& points, Octree& octree, Settings& settings);
void create_octree(Settings &settings, Octree &node, PointCloud& points);

void real_time_point_cloud_compression_eigentree(PointCloud& points, EigenTree& octree, Settings& settings);
void create_eigentree(Settings &settings, EigenTree &node, PointCloud& points);
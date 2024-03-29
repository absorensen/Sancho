#pragma once

#include "common_includes.h"

// project files
#include "camera.h"
#include "shader.h"
#include "point_cloud_loader.h"
#include "PointCloud.h"
#include "real_time_point_cloud_compression.h"
#include "performance_metrics.h"

// function declarations
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void process_input(GLFWwindow* window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void set_settings();


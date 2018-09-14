#version 400 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp_matrix;

uniform vec3 cam_pos;

uniform float point_size;
uniform float z_far;
uniform float z_near;
uniform float height_of_near_plane;

float linearize(float depth);


void main()
{
	gl_Position = mvp_matrix * vec4(aPos, 1.0);
//	gl_PointSize = point_size - (gl_Position.xyz- CamPos).length();
	gl_PointSize = (height_of_near_plane * point_size) / gl_Position.w;
}

float linearize(float depth)
{
	return -z_far * z_near / (depth * (z_far - z_near) - z_far);
}
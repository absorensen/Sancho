#version 400 core
layout (location = 0) in vec3 aPos;

uniform mat4 MVPmatrix;

uniform vec3 CamPos;

uniform float point_size;

void main()
{
	gl_Position = MVPmatrix * vec4(aPos, 1.0);
	gl_PointSize = point_size-(CamPos - aPos.xyz).length();
} 
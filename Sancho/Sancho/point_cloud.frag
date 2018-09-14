#version 400 core
out vec4 FragColor;

uniform vec3 color;
uniform vec3 cam_pos;

uniform float z_far;
uniform float z_near;

float linearize(float depth);

void main()
{
    FragColor = vec4(color, 1.0);
}

float linearize(float depth)
{
	return -z_far * z_near / (depth * (z_far - z_near) - z_far);
}
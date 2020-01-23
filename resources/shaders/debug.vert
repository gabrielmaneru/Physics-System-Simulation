#version 330 core

in vec3 attr_position;
in vec3 attr_color;
out vec3 vtx_color;

uniform mat4 vp;

void main()
{
	vtx_color = attr_color;
	gl_Position = vp * vec4(attr_position, 1.0f);
}
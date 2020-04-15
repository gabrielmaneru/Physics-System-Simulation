#version 330 core

layout(location = 0) in vec3 attr_position;
layout(location = 1) in vec3 attr_color;
layout(location = 2) in vec3 attr_normal;

out vec3 vtx_pos;
out vec3 vtx_color;
out vec3 vtx_norm;

uniform mat4 vp;

void main()
{
	vtx_pos = attr_position;
	vtx_color = attr_color;
	vtx_norm = attr_normal;
	gl_Position = vp * vec4(attr_position, 1.0f);
}
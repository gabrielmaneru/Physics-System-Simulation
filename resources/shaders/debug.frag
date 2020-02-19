#version 330 core

in vec3 vtx_color;
out vec4 out_color;
uniform float alpha;

void main()
{
	out_color = vec4(vtx_color, alpha);
}


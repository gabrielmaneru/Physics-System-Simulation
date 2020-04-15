#version 330 core

in vec3 vtx_pos;
in vec3 vtx_color;
in vec3 vtx_norm;

out vec4 out_color;

uniform float alpha;

void main()
{
	const vec3 light_diffuse = vec3(0.34,0.21,0.53);
	const vec3 ambient = vec3(0.56,0.79,0.47);
	const vec3 light_pos = vec3(0.0, 10.0, 10.0);
	
	vec3 norm = normalize(vtx_norm);
	vec3 l = normalize(light_pos - vtx_pos);

	l = normalize(l);
	float size = 20.0f;
	float d = floor(max(dot(norm, l),0.0)*size)/size;
	out_color = vec4((light_diffuse + d*ambient)*vtx_color, alpha);
}


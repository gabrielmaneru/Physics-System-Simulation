#include "math_utils.h"

glm::vec3 tr_point(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 1.0f));
}

glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 0.0f));
}

std::pair<glm::vec3, glm::vec3> closest_point_segments(const glm::vec3 & seg1A, const glm::vec3 & seg1B, const glm::vec3 & seg2A, const glm::vec3 & seg2B)
{
	const glm::vec3 d1 = seg1B - seg1A;
	const glm::vec3 d2 = seg2B - seg2A;
	const glm::vec3 r = seg1A - seg2A;
	const float a = glm::length2(d1);
	const float e = glm::length2(d2);

	if (a <= c_epsilon && e <= c_epsilon)
		return { seg1A, seg2A };

	const float f = glm::dot(d2, r);

	float s, t;
	if (a <= c_epsilon)
	{
		s = 0.0f;
		t = glm::clamp(f / e, 0.0f, 1.0f);
	}
	else
	{
		const float c = glm::dot(d1, r);

		if (e <= c_epsilon)
		{
			t = 0.0f;
			s = glm::clamp(-c / a, 0.0f, 1.0f);
		}
		else
		{
			const float b = glm::dot(d1, d2);
			const float denom = a * e - b * b;

			if (denom != 0.0f)
			{
				s = glm::clamp((b*f - c * e) / denom, 0.0f, 1.0f);
			}
			else
			{
				s = 0.0f;
			}

			t = (b * s + f) / e;

			if (t < 0.0f)
			{
				t = 0.0f;
				s = glm::clamp(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f)
			{
				t = 1.0f;
				s = glm::clamp((b - c) / a, 0.0f, 1.0f);
			}
		}
	}

	return { seg1A + d1 * s, seg2A + d2 * t };
}

float compute_seg_plane_intersection(const glm::vec3 & v1, const glm::vec3 & v2, float plane_d, const glm::vec3 & plane_n)
{
	const float nv1v2 = glm::dot(plane_n, v2 - v1);
	if (std::abs(nv1v2) > c_epsilon)
	{
		return (plane_d - glm::dot(plane_n, v1)) / nv1v2;
	}
	else
		return -1.f;
}

std::vector<glm::vec3> clip(const std::vector<glm::vec3>& vertices, const std::vector<std::pair<glm::vec3, glm::vec3>>& clipping_planes)
{
	std::vector<glm::vec3> input_vertices(vertices);
	std::vector<glm::vec3> output_vertices;

	for (auto p : clipping_planes)
	{
		output_vertices.clear();

		size_t input_count = input_vertices.size();
		size_t start = input_count - 1;

		for (size_t end = 0u; end < input_count; ++end)
		{
			const glm::vec3& v1 = input_vertices[start];
			const glm::vec3& v2 = input_vertices[end];

			const float v1N = glm::dot(v1 - p.second, p.first);
			const float v2N = glm::dot(v2 - p.second, p.first);

			if (v2N >= 0.0f)
			{
				if (v1N < 0.0f)
				{
					float t = compute_seg_plane_intersection(v1, v2, glm::dot(p.first, p.second), p.first);
					if(t >= 0.0f && t <= 1.0f)
						output_vertices.push_back(v1 + t * (v2 - v1));
					else
						output_vertices.push_back(v2);
				}
				output_vertices.push_back(v2);
			}
			else
			{
				if (v1N >= 0.0f)
				{
					float t = compute_seg_plane_intersection(v1, v2, -glm::dot(p.first, p.second), -p.first);
					if (t >= 0.0f && t <= 1.0f)
						output_vertices.push_back(v1 + t * (v2 - v1));
					else
						output_vertices.push_back(v1);
				}
			}
			start = end;
		}
		input_vertices = output_vertices;
	}
	return output_vertices;
}

glm::vec3 project_point_plane(const glm::vec3 & point, const glm::vec3 & normal, const glm::vec3 & plane_p)
{
	return point - glm::dot(normal, point - plane_p)*normal;
}

glm::vec3 make_ortho(const glm::vec3 n)
{
	const glm::vec3 n_abs = glm::abs(n);

	int min;
	if (n_abs.x < n_abs.z)
	{
		if (n_abs.x < n_abs.y)
			min = 0;
		else
			min = 1;
	}
	else
	{
		if (n_abs.y < n_abs.z)
			min = 1;
		else
			min = 2;
	}

	switch (min)
	{
	case 0:
		return glm::normalize(glm::vec3(0.0f, -n.z, n.y));
	case 1:
		return glm::normalize(glm::vec3(-n.z, 0.0f, n.x));
	case 2:
		return glm::normalize(glm::vec3(-n.y, n.x, 0.0f));
	}
	return{};
}

float rand01()
{
	return rand() / (float)RAND_MAX;
}

float rand(float a, float b)
{
	return a + (b - a)*rand01();
}

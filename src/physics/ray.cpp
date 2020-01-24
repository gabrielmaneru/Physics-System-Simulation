#include "ray.h"

glm::vec3 ray::get_point(float t) const
{
	return m_start + m_direction * t;
}

float ray::ray_cast_plane(glm::vec4 plane) const
{
	if (glm::length2(m_direction) > FLT_EPSILON)
	{
		glm::vec3 normal{ plane };
		float dot_ray_normal = glm::dot(m_direction, normal);
		if (std::abs(dot_ray_normal) > FLT_EPSILON)
		{
			glm::vec3 plane_point = normal * plane.w;
			glm::vec3 delta_pos = plane_point - m_start;
			return glm::dot(delta_pos, normal) / dot_ray_normal;
		}
	}
	return -1.0f;
}

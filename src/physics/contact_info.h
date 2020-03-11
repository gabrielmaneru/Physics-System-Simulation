#pragma once
#include <glm/glm.hpp>
#include "body.h"

struct contact_info
{
	bool m_hit{ false };
	glm::vec3 m_pi_A;
	glm::vec3 m_pi_B;
	glm::vec3 m_normal;
	float m_depth;
	body* m_body_A;
	body* m_body_B;

	contact_info() = default;
	contact_info(body* A, body* B)
		: m_hit{ true }, m_body_A{ A }, m_body_B{ B }
	{}
};
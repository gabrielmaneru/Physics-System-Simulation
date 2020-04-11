#include "contact_info.h"
#include "math_utils.h"
#include "body.h"

void overlap_pair::add_manifold(const contact_manifold & other)
{
	// Check if both manifold refer to the same normal
	// If have the same normal -> update points
	if (glm::dot(m_manifold.m_normal, other.m_normal) > 1.0f - c_epsilon)
	{
		contact_manifold next_manifold{ other };
		for (auto& p1 : next_manifold.m_points)
			for (auto p2 : m_manifold.m_points)
			{
				if (glm::length2(p1.m_local_A - p2.m_local_A) < c_epsilon
				 || glm::length2(p1.m_local_B - p2.m_local_B) < c_epsilon)
				{
					p1.impulse = p2.impulse;
					break;
				}
			}
		m_manifold = next_manifold;
	}
	// If the normal is different -> overwrite manifold
	else
		m_manifold = other;

	// Compute world points
	for (auto& p : m_manifold.m_points)
	{
		const glm::mat4 trAtoWorld = m_body_A->get_model();
		const glm::mat4 trBtoWorld = m_body_B->get_model();

		p.m_world_A = tr_point(trAtoWorld, p.m_local_A);
		p.m_world_B = tr_point(trBtoWorld, p.m_local_B);
	}
}

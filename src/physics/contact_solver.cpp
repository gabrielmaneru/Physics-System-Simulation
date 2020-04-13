#include "contact_solver.h"
#include "body.h"
#include "math_utils.h"

void constraint_contact_solver::evaluate(std::vector<overlap_pair*>& overlaps)
{
	// Warm start
	for (auto& o : overlaps)
		for (auto& p : o->m_manifold.m_points)
		{
			// Resting contact
			//if (p.impulse> 0.0f)
			//{
			//	const glm::vec3 dir_impulse = p.impulse * o->m_manifold.m_normal;
			//	
			//	if (!o->m_body_A->m_is_static)
			//		o->m_body_A->add_impulse(-dir_impulse, p.m_world_A);
			//	
			//	if (!o->m_body_B->m_is_static)
			//		o->m_body_B->add_impulse(dir_impulse, p.m_world_B);
			//}
			p.m_impulse = 0.0f;
		}

	// For each iteration
	for (int it = 0; it < m_iteration_count; ++it)
	{
		// For each pair of overlaps
		for (auto& pair : overlaps)
		{
			body* bA = pair->m_body_A;
			body* bB = pair->m_body_B;
			contact_manifold& manifold = pair->m_manifold;
			const glm::vec3& n = manifold.m_normal;

			for (auto& point : manifold.m_points)
			{
				const glm::vec3 vA = bA->get_linear_velocity();
				const glm::vec3 vB = bB->get_linear_velocity();
				const glm::vec3 wA = bA->get_angular_velocity();
				const glm::vec3 wB = bB->get_angular_velocity();
				const float JV = glm::dot((vB + glm::cross(wB, point.m_rB)) - (vA + glm::cross(wA, point.m_rA)), n);
				
				const float bias = -m_baumgarte * point.m_depth / physics_dt + point.m_restitutionBias;

				const float old = point.m_impulse;
				const float new_impulse = point.m_invEffMass * -(JV + bias);
				point.m_impulse = glm::max(point.m_impulse + new_impulse, 0.0f);
				const float delta_impulse = point.m_impulse - old;
				const glm::vec3 dir_impulse = delta_impulse * n;

				bA->add_impulse(-dir_impulse, point.m_pA);
				bB->add_impulse(dir_impulse, point.m_pB);
			}
		}
	}
}

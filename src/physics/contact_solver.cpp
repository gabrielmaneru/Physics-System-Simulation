#include "contact_solver.h"
#include "body.h"
#include "math_utils.h"

void naive_contact_solver::evaluate(std::vector<overlap_pair*>& overlaps)
{
	for (auto o : overlaps)
	for (auto p : o->m_manifold.m_points)
	{
		const glm::mat4 trAtoWorld = o->m_body_A->get_model();
		const glm::mat4 trBtoWorld = o->m_body_B->get_model();

		const glm::vec3 pA = tr_point(trAtoWorld, p.m_local_A);
		const glm::vec3 pB = tr_point(trBtoWorld, p.m_local_B);

		const glm::vec3 rA = pA - o->m_body_A->m_position;
		const glm::vec3 rB = pB - o->m_body_B->m_position;
		const glm::vec3 vA = o->m_body_A->get_velocity_at_point(pA);
		const glm::vec3 vB = o->m_body_B->get_velocity_at_point(pB);
		const float vRel = glm::dot(o->m_manifold.m_normal, vA - vB);

		if (vRel < 0.0f)
		{
			const bool any_static = o->m_body_A->m_is_static || o->m_body_B->m_is_static;
			const float rest_coeff = any_static ? 1.0f : 0.5f;
			const float jTop = -(1 + rest_coeff) * vRel;
			const float jBotA = o->m_body_A->m_is_static ? 0.0f : o->m_body_A->get_invmass()
				+ glm::dot(o->m_manifold.m_normal, glm::cross(o->m_body_A->get_oriented_invinertia() * glm::cross(rA, o->m_manifold.m_normal), rA));
			const float jBotB = o->m_body_B->m_is_static ? 0.0f : o->m_body_B->get_invmass()
				+ glm::dot(o->m_manifold.m_normal, glm::cross(o->m_body_B->get_oriented_invinertia() * glm::cross(rB, o->m_manifold.m_normal), rB));
			const float jBot = jBotA + jBotB;

			if (jBot != 0.0f)
				p.m_impulse = jTop / jBot;
		}
	}

	for (auto o : overlaps)
	for (auto p : o->m_manifold.m_points)
	{
		const glm::mat4 trAtoWorld = o->m_body_A->get_model();
		const glm::mat4 trBtoWorld = o->m_body_B->get_model();

		const glm::vec3 pA = tr_point(trAtoWorld, p.m_local_A);
		const glm::vec3 pB = tr_point(trBtoWorld, p.m_local_B);

		const glm::vec3 force = p.m_impulse * o->m_manifold.m_normal;

		if (!o->m_body_A->m_is_static)
			o->m_body_A->add_impulse(force, pA);

		if (!o->m_body_B->m_is_static)
			o->m_body_B->add_impulse(-force, pB);
	}
}

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
			contact_manifold& manifold = pair->m_manifold;
			const glm::vec3& n = manifold.m_normal;
			const glm::vec3& vA = pair->m_body_A->m_linear_momentum;
			const glm::vec3& vB = pair->m_body_B->m_linear_momentum;
			const glm::vec3& wA = pair->m_body_A->m_angular_momentum;
			const glm::vec3& wB = pair->m_body_B->m_angular_momentum;

			for (auto& point : manifold.m_points)
			{

				const glm::vec3 deltaV = vB + glm::cross(wB, point.m_rB) - vA - glm::cross(wA, point.m_rA);
				const float JV = glm::dot(deltaV, n);

				const float bias_penetration = point.m_depth > c_slop ? -m_baumgarte / physics_dt * (point.m_depth - c_slop) : 0.0f;
				const float b = bias_penetration + point.m_restitutionBias;

				float delta_lambda = point.m_invEffMass * -(JV + b);
				const float old = point.m_impulse;
				point.m_impulse = glm::max(point.m_impulse + delta_lambda, 0.0f);
				delta_lambda = point.m_impulse - old;
				const glm::vec3 dir_impulse = delta_lambda * n;

				pair->m_body_A->add_impulse(-dir_impulse, point.m_pA);
				pair->m_body_B->add_impulse( dir_impulse, point.m_pB);
			}
		}
	}
}

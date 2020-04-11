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
				p.impulse = jTop / jBot;
		}
	}

	for (auto o : overlaps)
	for (auto p : o->m_manifold.m_points)
	{
		const glm::mat4 trAtoWorld = o->m_body_A->get_model();
		const glm::mat4 trBtoWorld = o->m_body_B->get_model();

		const glm::vec3 pA = tr_point(trAtoWorld, p.m_local_A);
		const glm::vec3 pB = tr_point(trBtoWorld, p.m_local_B);

		const glm::vec3 force = p.impulse * o->m_manifold.m_normal;

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
			p.impulse = 0.0f;
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
				const glm::vec3 rA = point.m_world_A - bA->m_position;
				const glm::vec3 rB = point.m_world_B - bB->m_position;
				const glm::vec3 rAxN = glm::cross(rA, n);
				const glm::vec3 rBxN = glm::cross(rB, n);
				const glm::vec3 IxrAxN = bA->get_oriented_invinertia() * rAxN;
				const glm::vec3 IxrBxN = bB->get_oriented_invinertia() * rBxN;
				const float effM
					= bA->get_invmass()
					+ bB->get_invmass()
					+ glm::dot(rAxN, IxrAxN)
					+ glm::dot(rBxN, IxrBxN);
				const float invEffM = effM > 0.0f ? 1.0f / effM : 0.0f;

				const glm::vec3 vA = bA->get_velocity_at_point(point.m_world_A);
				const glm::vec3 vB = bB->get_velocity_at_point(point.m_world_B);
				const glm::vec3 wA = bA->get_angular_velocity();
				const glm::vec3 wB = bB->get_angular_velocity();
				const float JV = glm::dot((vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA)), n);
				if (it == 0)
					point.JV0 = JV;

				const float bias = -m_baumgarte * point.m_depth / physics_dt + m_restitution * point.JV0;

				const float old = point.impulse;
				const float new_impulse = invEffM * -(JV + bias);
				point.impulse = glm::max(point.impulse + new_impulse, 0.0f);
				const float delta_impulse = point.impulse - old;
				const glm::vec3 dir_impulse = delta_impulse * n;

				bA->add_impulse(-dir_impulse, point.m_world_A);
				bB->add_impulse( dir_impulse, point.m_world_B);
			}
		}
	}
}

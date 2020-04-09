#include "contact_solver.h"

void naive_contact_solver::evaluate(std::vector<contact_point>& contacts)
{
	for (contact_point& c : contacts)
	{
		const glm::vec3 rA = c.m_pi_A - c.m_body_A->m_position;
		const glm::vec3 rB = c.m_pi_B - c.m_body_B->m_position;
		const glm::vec3 vA = c.m_body_A->get_velocity_at_point(c.m_pi_A);
		const glm::vec3 vB = c.m_body_B->get_velocity_at_point(c.m_pi_B);
		const float vRel = glm::dot(c.m_normal, vA - vB);

		if (vRel < 0.0f)
		{
			const bool any_static = c.m_body_A->m_is_static || c.m_body_B->m_is_static;
			const float rest_coeff = any_static? 1.0f : 0.5f;
			const float jTop = -(1 + rest_coeff) * vRel;
			const float jBotA = c.m_body_A->m_is_static ? 0.0f : c.m_body_A->get_invmass()
				+ glm::dot(c.m_normal, glm::cross(c.m_body_A->get_oriented_invinertia() * glm::cross(rA, c.m_normal), rA));
			const float jBotB = c.m_body_B->m_is_static ? 0.0f : c.m_body_B->get_invmass()
				+ glm::dot(c.m_normal, glm::cross(c.m_body_B->get_oriented_invinertia() * glm::cross(rB, c.m_normal), rB));
			const float jBot = jBotA + jBotB;

			if (jBot != 0.0f)
				c.impulse = jTop/jBot;
		}
	}

	for (contact_point& c : contacts)
	{
		const glm::vec3 force = c.impulse * c.m_normal;

		if (!c.m_body_A->m_is_static)
			c.m_body_A->add_impulse(force, c.m_pi_A);

		if (!c.m_body_B->m_is_static)
			c.m_body_B->add_impulse(-force, c.m_pi_B);
	}
}

void constraint_contact_solver::evaluate(std::vector<contact_point>& contacts)
{
	for(int it = 0; it < m_iteration_count; ++it)
		for (contact_point& c : contacts)
		{
			const glm::vec3 rA = c.m_pi_A - c.m_body_A->m_position;
			const glm::vec3 rB = c.m_pi_B - c.m_body_B->m_position;
			const glm::vec3 rAxN = glm::cross(rA, c.m_normal);
			const glm::vec3 rBxN = glm::cross(rB, c.m_normal);
			const glm::vec3 IxrAxN = c.m_body_A->get_oriented_invinertia() * rAxN;
			const glm::vec3 IxrBxN = c.m_body_B->get_oriented_invinertia() * rBxN;
			const float effM
				= c.m_body_A->get_invmass()
				+ c.m_body_B->get_invmass()
				+ glm::dot(rAxN, IxrAxN)
				+ glm::dot(rBxN, IxrBxN);
			const float invEffM = effM > 0.0f? 1.0f / effM : 0.0f;

			const glm::vec3 vA = c.m_body_A->get_velocity_at_point(c.m_pi_A);
			const glm::vec3 vB = c.m_body_B->get_velocity_at_point(c.m_pi_B);
			const glm::vec3 wA = c.m_body_A->get_angular_velocity();
			const glm::vec3 wB = c.m_body_B->get_angular_velocity();
			const float JV = glm::dot( (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA)), c.m_normal);
			if (it == 0)
				c.JV0 = JV;

			const float bias = -m_baumgarte * c.m_depth / physics_dt + m_restitution * c.JV0;
			
			const float old = c.impulse;
			const float new_impulse = invEffM * -(JV + bias);
			c.impulse = glm::max(c.impulse + new_impulse,0.0f);
			const float delta_impulse = c.impulse - old;
			const glm::vec3 dir_impulse = delta_impulse * c.m_normal;

			if (!c.m_body_A->m_is_static)
				c.m_body_A->add_impulse(-dir_impulse, c.m_pi_A);

			if (!c.m_body_B->m_is_static)
				c.m_body_B->add_impulse(dir_impulse, c.m_pi_B);
		}
}

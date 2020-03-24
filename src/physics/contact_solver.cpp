#include "contact_solver.h"

void naive_contact_solver::evaluate(std::vector<contact>& contacts)
{
	for (contact& c : contacts)
	{
		const glm::vec3 rA = c.m_pi_A - c.m_body_A->m_position;
		const glm::vec3 rB = c.m_pi_B - c.m_body_B->m_position;
		const glm::vec3 vA = c.m_body_A->get_point_velocity(c.m_pi_A);
		const glm::vec3 vB = c.m_body_B->get_point_velocity(c.m_pi_B);
		const float vRel = glm::dot(c.m_normal, vA - vB);

		if (vRel < 0.0f)
		{
			const float rest_coeff = 1.0f;
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

	for (contact& c : contacts)
	{
		const glm::vec3 force = c.impulse * c.m_normal;

		if (!c.m_body_A->m_is_static)
			c.m_body_A->add_force(force, c.m_pi_A);

		if (!c.m_body_B->m_is_static)
			c.m_body_B->add_force(-force, c.m_pi_B);
	}
}

void constraint_contact_solver::evaluate(std::vector<contact>& contacts)
{
	for(int i=0;i< m_iteration_count; ++i)
		for (contact& c : contacts)
		{
			const glm::vec3 rA = c.m_pi_A - c.m_body_A->m_position;
			const glm::vec3 rB = c.m_pi_B - c.m_body_B->m_position;
			const glm::vec3 vA = c.m_body_A->get_point_velocity(c.m_pi_A);
			const glm::vec3 vB = c.m_body_B->get_point_velocity(c.m_pi_B);
			const float vRel = glm::dot(c.m_normal, vA - vB);
			if (vRel < 0.0f)
			{
				const float rest_coeff = 0.0f;
				const float jTop = -(1 + rest_coeff) * vRel;
				const float jBotA = c.m_body_A->m_is_static ? 0.0f : c.m_body_A->get_invmass()
					+ glm::dot(c.m_normal, glm::cross(c.m_body_A->get_oriented_invinertia() * glm::cross(rA, c.m_normal), rA));
				const float jBotB = c.m_body_B->m_is_static ? 0.0f : c.m_body_B->get_invmass()
					+ glm::dot(c.m_normal, glm::cross(c.m_body_B->get_oriented_invinertia() * glm::cross(rB, c.m_normal), rB));
				const float jBot = jBotA + jBotB;

				if (jBot != 0.0f)
					c.impulse = jTop / jBot;
			}
		}

	for (contact& c : contacts)
	{
		const glm::vec3 force = c.impulse * c.m_normal;
		c.m_body_A->add_force(force, c.m_pi_A);
		c.m_body_B->add_force(-force, c.m_pi_B);
	}
}

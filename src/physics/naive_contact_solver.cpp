#include "naive_contact_solver.h"

void naive_contact_solver::evaluate(const std::vector<contact_info>& contacts)
{
	for (const contact_info& c : contacts)
	{
		const glm::vec3 rA = c.m_pi_A - c.m_body_A->m_position;
		const glm::vec3 rB = c.m_pi_B - c.m_body_B->m_position;
		const glm::vec3 vA = c.m_body_A->get_point_velocity(c.m_pi_A);
		const glm::vec3 vB = c.m_body_B->get_point_velocity(c.m_pi_B);
		const float vRel = glm::dot(c.m_normal, vA - vB);

		const float rest_coeff = 1.0f;
		const float jTop = -(1 + rest_coeff) * vRel;
		const float jBot
			= c.m_body_A->get_invmass()
			+ c.m_body_B->get_invmass()
			+ glm::dot(c.m_normal, glm::cross(c.m_body_A->get_oriented_invinertia() * glm::cross(rA, c.m_normal), rA))
			+ glm::dot(c.m_normal, glm::cross(c.m_body_B->get_oriented_invinertia() * glm::cross(rB, c.m_normal), rB));
		const float j = jTop/jBot;

		const glm::vec3 force = j * c.m_normal;
		c.m_body_A->add_force(force, rA);
		c.m_body_B->add_force(force, rB);
	}
}

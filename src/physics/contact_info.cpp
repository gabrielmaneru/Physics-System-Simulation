#include "contact_info.h"
#include "math_utils.h"
#include "body.h"

overlap_pair::overlap_pair(body * bA, body * bB, const physical_mesh * mA, const physical_mesh * mB)
	:m_body_A(bA),m_body_B(bB), m_mesh_A(mA), m_mesh_B(mB)
{
	m_manifold.m_friction_coef = std::sqrtf(bA->m_friction_coef * bB->m_friction_coef);
	m_manifold.m_rolling_coef = std::sqrtf(bA->m_rolling_coef * bB->m_rolling_coef);
	m_manifold.m_restitution_coef = std::fmax(bA->m_bounciness_coef, bB->m_bounciness_coef);

	m_manifold.m_invM_A = bA->get_invmass();
	m_manifold.m_invM_B = bB->get_invmass();
	m_manifold.m_invI_A = bA->get_oriented_invinertia();
	m_manifold.m_invI_B = bB->get_oriented_invinertia();
}

void overlap_pair::update()
{
	// Get data
	const glm::vec3& n = m_manifold.m_normal;
	const glm::vec3& xA = m_body_A->m_position;
	const glm::vec3& xB = m_body_B->m_position;
	const glm::vec3& vA = m_body_A->get_linear_velocity();
	const glm::vec3& vB = m_body_B->get_linear_velocity();
	const glm::vec3& wA = m_body_A->get_angular_velocity();
	const glm::vec3& wB = m_body_B->get_angular_velocity();
	const glm::mat4 AtoW = m_body_A->get_model();
	const glm::mat4 BtoW = m_body_B->get_model();

	// Update each contact point
	for (auto& p : m_manifold.m_points)
	{
		// Position vectors
		p.m_pA = tr_point(AtoW, p.m_local_A);
		p.m_pB = tr_point(BtoW, p.m_local_B);
		p.m_rA = p.m_pA - xA;
		p.m_rB = p.m_pB - xB;

		// Precompute vectors
		const glm::vec3 rAxN = glm::cross(p.m_rA, n);
		const glm::vec3 rBxN = glm::cross(p.m_rB, n);
		p.m_iAxrAxN = m_manifold.m_invI_A * rAxN;
		p.m_iBxrBxN = m_manifold.m_invI_B * rBxN;

		// Compute effective mass
		const float eff_mass = m_manifold.m_invM_A + m_manifold.m_invM_B + glm::dot(glm::cross(p.m_iAxrAxN, p.m_rA), n) + glm::dot(glm::cross(p.m_iBxrBxN, p.m_rB), n);
		p.m_invEffMass = eff_mass > 0.0f ? 1.f / eff_mass : 0.0f;

		// Compute JV0
		const glm::vec3 deltaV = vB + glm::cross(wB, p.m_rB) - vA - glm::cross(wA, p.m_rA);
		const float JV0 = glm::dot(deltaV, n);

		// Compute restitution bias
		p.m_restitutionBias = JV0 < -c_rest_vel_threshold ? JV0 * m_manifold.m_restitution_coef : 0.0f;
	}
}

void overlap_pair::add_manifold(const simple_manifold & other)
{
	// Check if both manifold refer to the same normal
	// If have the same normal -> update points
	if (glm::dot(m_manifold.m_normal, other.m_normal) > 1.0f - c_epsilon)
	{
		std::vector<contact_point> points = other.m_points;
		for (auto& p1 : points)
			for (auto p2 : m_manifold.m_points)
			{
				if (glm::length2(p1.m_local_A - p2.m_local_A) < c_epsilon
					|| glm::length2(p1.m_local_B - p2.m_local_B) < c_epsilon)
				{
					p1.m_impulse = p2.m_impulse;
					break;
				}
			}
		m_manifold.m_points = points;
		m_manifold.m_normal = glm::normalize(other.m_normal);
	}
	// If the normal is different -> overwrite manifold
	else
	{
		m_manifold.m_normal = glm::normalize(other.m_normal);
		m_manifold.m_points = other.m_points;
	}
}

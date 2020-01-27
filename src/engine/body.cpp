#include "body.h"

void body::integrate(float dt)
{
	// Add Forces to Linear Momentum
	m_linear_momentum += m_forces_accumulation;
	m_forces_accumulation = glm::vec3{ 0.0f };

	// Apply velocity
	glm::vec3 v = m_linear_momentum / m_mass;
	m_position += v * dt;

	// Add Torques to Angular Momentum
	m_angular_momentum += m_torques_accumulation;
	m_torques_accumulation = glm::vec3{ 0.0f };

	// Apply rotation
	glm::vec3 w = m_inertia_inv * m_angular_momentum;
	glm::quat qw{ 0.0f, w.x, w.y, w.z };
	glm::quat d_R = .5f * (qw * m_rotation);
	m_rotation = glm::normalize(m_rotation + d_R * dt);

}

body & body::set_inertia(glm::mat3 i)
{
	m_inertia = i;
	m_inertia_inv = glm::inverse(i);
	return *this;
}

void body::add_force(glm::vec3 force, glm::vec3 point)
{
	glm::vec3 R = point - m_mass_center;
	R = tr_vector(glm::inverse(get_model()), R);

	m_forces_accumulation += force;

	float d = glm::dot(glm::normalize(force), glm::normalize(R));
	if (glm::abs(d) < 1.0f - c_epsilon)
		m_torques_accumulation += glm::cross(R, force);
}

glm::mat4 body::get_model()const
{
	return glm::translate(glm::mat4(1.0f), m_position) * glm::mat4_cast(m_rotation);
}

glm::vec3 tr_point(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 1.0f));
}

glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 0.0f));
}

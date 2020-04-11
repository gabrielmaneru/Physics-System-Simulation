#pragma once
#include <glm/glm.hpp>
#include <vector>

struct body;
struct physical_mesh;

struct contact_point
{
	glm::vec3 m_local_A;
	glm::vec3 m_local_B;
	float m_depth;
	float m_impulse{ 0.0f };

	glm::vec3 m_pA;
	glm::vec3 m_pB;
	glm::vec3 m_rA;
	glm::vec3 m_rB;
	glm::vec3 m_iAxrAxN;
	glm::vec3 m_iBxrBxN;
	float m_invEffMass;
	float m_restitutionBias;
};

struct simple_manifold
{
	glm::vec3 m_normal;
	std::vector<contact_point> m_points;
};
struct contact_manifold : public simple_manifold
{
	float m_friction_coef;
	float m_rolling_coef;
	float m_restitution_coef;

	float m_invM_A;
	float m_invM_B;
	glm::mat3 m_invI_A;
	glm::mat3 m_invI_B;
};

struct overlap_pair
{

	enum class state {
		New, NoCollision, Collision
	}m_state{state::New};
	body* m_body_A;
	body* m_body_B;
	const physical_mesh* m_mesh_A;
	const physical_mesh* m_mesh_B;
	contact_manifold m_manifold;

	overlap_pair() = default;
	overlap_pair(body* bA, body* bB, const physical_mesh* mA, const physical_mesh* mB);
	void update();
	void add_manifold(const simple_manifold& other);
};
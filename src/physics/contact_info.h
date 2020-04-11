#pragma once
#include <glm/glm.hpp>
#include <list>

struct body;
struct physical_mesh;

struct contact_point
{
	glm::vec3 m_local_A;
	glm::vec3 m_local_B;
	glm::vec3 m_world_A;
	glm::vec3 m_world_B;
	float m_depth;
	float impulse{ 0.0f };
	float JV0;
};

struct contact_manifold
{
	glm::vec3 m_normal;
	std::list<contact_point> m_points;
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

	void add_manifold(const contact_manifold& other);
};
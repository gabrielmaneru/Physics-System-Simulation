#pragma once
#include <glm/glm.hpp>
#include <list>

struct body;
struct physical_mesh;

struct contact_point
{
	glm::vec3 m_pointA;
	glm::vec3 m_pointB;
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
	body* m_body_A;
	body* m_body_B;
	const physical_mesh* m_mesh_A;
	const physical_mesh* m_mesh_B;
	contact_manifold m_manifold;
};
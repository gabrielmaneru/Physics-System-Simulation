#pragma once
#include <glm/glm.hpp>
#include <vector>

struct body;
struct physical_mesh;

struct contact_point
{
	glm::vec3 local_A;
	glm::vec3 local_B;
	float depth;

	glm::vec3 point_A;
	glm::vec3 point_B;
	float lambda_Vel{ 0.0f };
	float invM_Vel;
	float restitution_bias;
};

struct simple_manifold
{
	glm::vec3 normal;
	std::vector<contact_point> points;
};
struct contact_manifold : public simple_manifold
{
	float coef_friction;
	float coef_roll;
	float coef_restitution;

	glm::mat3 invI_A;
	glm::mat3 invI_B;
	float     invM_A;
	float     invM_B;
	float     invM_U;
	float     invM_V;
	float     invM_Twist;
	glm::mat3 invM_Roll;

	float     lambda_U{0.0f};
	float     lambda_V{0.0f};
	float     lambda_Twist{0.0f};
	glm::vec3 lambda_Roll{0.0f, 0.0f, 0.0f};

	glm::vec3 avg_point_A;
	glm::vec3 avg_point_B;
	glm::vec3 vec_U;
	glm::vec3 vec_V;
	glm::vec3 oldvec_U;
	glm::vec3 oldvec_V;
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
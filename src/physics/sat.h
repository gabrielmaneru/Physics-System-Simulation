#pragma once
#include <glm/glm.hpp>
#include <vector>

struct physical_mesh;
struct body;
struct face;
struct half_edge;
struct overlap_pair;

struct sat
{
	enum class actor { A, B, Edge, Null };
	struct penetration_data
	{
		actor m_actor;
		float m_penetration{ FLT_MAX };
		const face* m_face{ nullptr };
		const half_edge* m_edgeA{ nullptr };
		const half_edge* m_edgeB{nullptr};
	};
	struct simple_manifold
	{
		glm::vec3 normal;
		std::vector<glm::vec3> local_A;
		std::vector<glm::vec3> local_B;
	};
	struct result
	{
		bool m_contact{ false };
		simple_manifold m_manifold;
	};

	sat(const overlap_pair * pair);
	result test_collision();
	
private:

	const body* bA;
	const body* bB;
	const physical_mesh* mA;
	const physical_mesh* mB;
	const glm::mat4 trAtoWorld;;
	const glm::mat4 trBtoWorld;
	const glm::mat4 trAtoB;;
	const glm::mat4 trBtoA;
	const bool was_colliding;
	const penetration_data& prev_data;
	penetration_data& next_data;

	glm::vec3 edge_data[4];

	penetration_data test_faces(actor);
	float compute_face_penetration(const physical_mesh * other, const glm::mat4 tr, const face * face);

	penetration_data test_edges();
	bool test_gaussmap_intersect(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d, const glm::vec3& bCrossA, const glm::vec3& dCrossC)const;
	float compute_edge_penetration(const glm::vec3& edge1_start, const glm::vec3& edge2_start, const glm::vec3& centroidA, const glm::vec3& edge1_dir, const glm::vec3& edge2_dir);

	result generate_manifold(const penetration_data& data);
};
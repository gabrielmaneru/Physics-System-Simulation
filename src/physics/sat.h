#pragma once
#include "contact_info.h"

struct physical_mesh;
struct face;
struct half_edge;

struct sat
{
	enum class actor { A, B, Edge };
	struct penetration_data
	{
		actor m_actor;
		float m_penetration{ FLT_MAX };
		union {
			const face* m_face;
			struct {
				const half_edge* A;
				const half_edge* B;
			}m_edges;
		}m_pointers;
	};
	struct result
	{
		bool m_contact{ false };
		contact_manifold m_manifold;
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

	glm::vec3 edge_data[4];

	penetration_data test_faces(actor);
	float compute_face_penetration(const physical_mesh * other, const glm::mat4 tr, const face * face);

	penetration_data test_edges();
	bool test_gaussmap_intersect(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d, const glm::vec3& bCrossA, const glm::vec3& dCrossC)const;
	float compute_edge_penetration(const glm::vec3& edge1_start, const glm::vec3& edge2_start, const glm::vec3& centroidA, const glm::vec3& edge1_dir, const glm::vec3& edge2_dir);

	contact_manifold generate_manifold(const penetration_data& data);
};
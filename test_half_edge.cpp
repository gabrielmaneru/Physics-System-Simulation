#include <gtest/gtest.h>
#include "physical_mesh.h"

TEST(half_edge, half_edge_triangle)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
	};

	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });

	ASSERT_EQ(mesh.m_vertices.size(), 4u);
	ASSERT_EQ(mesh.m_hedges.size(), 6u);
	ASSERT_EQ(mesh.m_faces.size(), 2u);
}

TEST(half_edge, half_edge_quad)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
	};

	mesh.add_face({ 0u,1u,2u,3u });

	ASSERT_EQ(mesh.m_vertices.size(), 4u);
	ASSERT_EQ(mesh.m_hedges.size(), 4u);
	ASSERT_EQ(mesh.m_faces.size(), 1u);
}

TEST(half_edge, half_edge_twin)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
	};

	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	mesh.create_twins();

	auto it = mesh.m_hedges.cbegin();
	ASSERT_TRUE(it++->m_twin != nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it++->m_twin != nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
}

TEST(half_edge, half_edge_merge_coplanar)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
	};

	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	mesh.create_twins();
	mesh.merge_coplanar();

	ASSERT_TRUE(false);
}

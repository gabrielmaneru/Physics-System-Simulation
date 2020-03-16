#include <gtest/gtest.h>

// Half Edge Mesh
#include <physics/physical_mesh.h>
TEST(half_edge, half_edge_triangle)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f}
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
		glm::vec3{0.0f, 1.0f, 0.0f}
	};
	mesh.add_face({ 0u,1u,2u,3u });

	ASSERT_EQ(mesh.m_vertices.size(), 4u);
	ASSERT_EQ(mesh.m_hedges.size(), 4u);
	ASSERT_EQ(mesh.m_faces.size(), 1u);
}
TEST(half_edge, half_edge_create_twin_1)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f}
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
	ASSERT_TRUE(it->m_twin == nullptr);
}
TEST(half_edge, half_edge_create_twin_2)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
		glm::vec3{0.5f, 1.5f, 1.0f}
	};
	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	mesh.add_face({ 2u,4u,3u });
	mesh.create_twins();

	auto it = mesh.m_hedges.cbegin();
	ASSERT_TRUE(it++->m_twin != nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it++->m_twin != nullptr);
	ASSERT_TRUE(it++->m_twin != nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it++->m_twin != nullptr);
	ASSERT_TRUE(it++->m_twin == nullptr);
	ASSERT_TRUE(it->m_twin == nullptr);
}
TEST(half_edge, half_edge_is_coplanar)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
		glm::vec3{0.5f, 1.5f, 1.0f}
	};
	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	mesh.add_face({ 2u,4u,3u });
	mesh.create_twins();

	auto it = mesh.m_hedges.cbegin();
	ASSERT_TRUE(mesh.is_coplanar(&*it)); it++; it++; it++;
	ASSERT_TRUE(mesh.is_coplanar(&*it)); it++;
	ASSERT_FALSE(mesh.is_coplanar(&*it)); it++; it++;
	ASSERT_FALSE(mesh.is_coplanar(&*it));
}
TEST(half_edge, half_edge_merge_coplanar)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f}
	};
	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	mesh.create_twins();
	mesh.merge_coplanar();

	ASSERT_TRUE(mesh.m_faces.size() == 1);
	ASSERT_TRUE(mesh.m_faces.front().m_indices.size() == 4);
	ASSERT_TRUE(mesh.m_hedges.size() == 4);
}
TEST(half_edge, half_edge_merge_coplanar_full_face)
{
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f},
		glm::vec3{0.5f, 0.5f, 0.0f}
	};
	mesh.add_face({ 0u,1u,4u });
	mesh.add_face({ 1u,2u,4u });
	mesh.add_face({ 2u,3u,4u });
	mesh.add_face({ 3u,0u,4u });
	mesh.create_twins();
	mesh.merge_coplanar();

	ASSERT_TRUE(mesh.m_faces.size() == 1);
	ASSERT_TRUE(mesh.m_faces.front().m_indices.size() == 4);
	ASSERT_TRUE(mesh.m_hedges.size() == 4);
}

// Solver
#include <physics/body.h>
#include <physics/naive_contact_solver.h>
TEST(solver, contact_naive)
{
	body a;
	a.m_linear_momentum = glm::vec3{ 0,-1,0 };
	a.m_position = glm::vec3{ 0,0,0 };
	a.set_mass(1.0f);
	a.set_inertia(glm::mat3{ 1.0f });

	body b;
	b.set_static(true);

	contact_info c;
	c.m_body_A = &a;
	c.m_body_B = &b;
	c.m_normal = glm::normalize(glm::vec3{ 1,1,0 });
	c.m_pi_A = c.m_pi_B = a.m_position - c.m_normal*5.0f;

	naive_contact_solver{}.evaluate({ c });
	a.integrate(0.f);
	b.integrate(0.f);

	ASSERT_TRUE(glm::all(glm::epsilonEqual(a.get_linear_velocity(), { 1, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(a.get_angular_velocity(), { 0, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(b.get_linear_velocity(), { 0, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(b.get_angular_velocity(), { 0, 0, 0 }, 0.001f)));
}
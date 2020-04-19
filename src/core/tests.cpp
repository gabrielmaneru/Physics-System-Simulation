/**
 * @file tests.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Tests list
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include <gtest/gtest.h>

// Half Edge Mesh
#include <physics/physical_mesh.h>
TEST(half_edge, half_edge_triangle)
{
	// Create Mesh
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f}
	};
	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	// Test mesh
	ASSERT_EQ(mesh.m_vertices.size(), 4u);
	ASSERT_EQ(mesh.m_hedges.size(), 6u);
	ASSERT_EQ(mesh.m_faces.size(), 2u);
}
TEST(half_edge, half_edge_quad)
{
	// Create Mesh
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f}
	};
	mesh.add_face({ 0u,1u,2u,3u });
	// Test mesh
	ASSERT_EQ(mesh.m_vertices.size(), 4u);
	ASSERT_EQ(mesh.m_hedges.size(), 4u);
	ASSERT_EQ(mesh.m_faces.size(), 1u);
}
TEST(half_edge, half_edge_create_twin_1)
{
	// Create Mesh
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
	// Test mesh
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
	// Create Mesh
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
	// Create mesh twins
	mesh.create_twins();
	// Test mesh
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
	// Create Mesh
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
	// Create mesh twins
	mesh.create_twins();
	// Test mesh
	auto it = mesh.m_hedges.cbegin();
	ASSERT_TRUE(mesh.is_coplanar(&*it)); it++; it++; it++;
	ASSERT_TRUE(mesh.is_coplanar(&*it)); it++;
	ASSERT_FALSE(mesh.is_coplanar(&*it)); it++; it++;
	ASSERT_FALSE(mesh.is_coplanar(&*it));
}
TEST(half_edge, half_edge_merge_coplanar)
{
	// Create Mesh
	physical_mesh mesh{};
	mesh.m_vertices = {
		glm::vec3{0.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 0.0f, 0.0f},
		glm::vec3{1.0f, 1.0f, 0.0f},
		glm::vec3{0.0f, 1.0f, 0.0f}
	};
	mesh.add_face({ 0u,1u,2u });
	mesh.add_face({ 2u,3u,0u });
	// Create mesh twins
	mesh.create_twins();
	// Merge coplanar faces
	mesh.merge_coplanar();
	// Test mesh
	ASSERT_TRUE(mesh.m_faces.size() == 1);
	ASSERT_TRUE(mesh.m_faces.front().m_indices.size() == 4);
	ASSERT_TRUE(mesh.m_hedges.size() == 4);
}
TEST(half_edge, half_edge_merge_coplanar_full_face)
{
	// Create Mesh
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
	// Create mesh twins
	mesh.create_twins();
	// Merge coplanar faces
	mesh.merge_coplanar();
	// Test mesh
	ASSERT_TRUE(mesh.m_faces.size() == 1);
	ASSERT_TRUE(mesh.m_faces.front().m_indices.size() == 4);
	ASSERT_TRUE(mesh.m_hedges.size() == 4);
}

// Solver
#include <physics/body.h>
#include <physics/contact_solver.h>
TEST(naive_solver, simple_test)
{
	// Create static body A
	body a;
	a.set_static(true);
	a.set_restitution(1.0f);
	// Create dynamic body B
	body b;
	b.m_linear_momentum = glm::vec3{ 0,-1,0 };
	b.set_position(glm::vec3{});
	b.set_inertia(glm::mat3{ 1.0f });
	b.set_restitution(1.0f);
	// Create pair
	overlap_pair pair{ &a, &b, nullptr, nullptr };
	pair.manifold.normal = glm::normalize(glm::vec3{ 1,1,0 });
	glm::vec3 pi = a.m_position - pair.manifold.normal*5.0f;
	pair.manifold.points.emplace_back(contact_point{ pi,pi });
	pair.update();
	pair.manifold.points[0].invM_Vel = 2.0f;
	//Fill overlap vector
	std::vector<overlap_pair*> pairs{&pair};
	// Solve
	constraint_contact_solver{ 1, 0.0f }.evaluate(pairs);
	// Test solver
	ASSERT_NEAR(pair.manifold.points[0].lambda_Vel, glm::sqrt(2.0f), 0.001f);
	ASSERT_TRUE(glm::all(glm::epsilonEqual(b.get_linear_velocity(), { 1, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(b.get_angular_velocity(), { 0, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(a.get_linear_velocity(), { 0, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(a.get_angular_velocity(), { 0, 0, 0 }, 0.001f)));
}
TEST(constraint_solver, circle_single)
{
	// Define general speed
	float speed = 0.1f;
	// Create static body A
	body a;
	a.set_static(true);
	// Create dynamic body B
	body b;
	b.set_mass(1.0f);
	b.set_inertia(glm::mat3{ 1.0f / 6.0f });
	b.set_position(glm::vec3{ 0, 0.5f, 0 });
	b.m_linear_momentum = { 0, -speed, 0 };
	// Create pair
	overlap_pair pair{ &a, &b, nullptr, nullptr };
	pair.manifold.normal = { 0,1,0 };
	pair.manifold.points.push_back(contact_point{ glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -0.5f, 0.0f) });
	pair.update();
	//Fill overlap vector
	std::vector<overlap_pair *> pairs { &pair };
	// Solve
	constraint_contact_solver{ 256, 0.0f }.evaluate(pairs);
	// Test solver
	float supporting_mass = b.get_mass();
	ASSERT_NEAR(pair.manifold.points[0].lambda_Vel, supporting_mass * speed, 0.0001f);
	ASSERT_NEAR(b.m_linear_momentum.y, 0.0f, 0.0001f);
	ASSERT_NEAR(glm::length2(b.m_linear_momentum), 0.0f, 0.0001f);
	ASSERT_NEAR(glm::length2(b.m_angular_momentum), 0.0f, 0.0001f);
}
#include <numeric>
TEST(constraint_solver, circle_stack)
{
	// Define general speed
	float spd = 0.1f;
	// Define body count
	std::vector<body> bodies(5);
	// Create static body
	bodies[0].set_static(true);
	// Create dynamic bodies
	for (size_t i = 1; i < bodies.size(); ++i)
	{
		body& b = bodies[i];
		b.set_mass(1.0f);
		b.set_inertia(glm::mat3{ 1.0f / 6.0f });
		b.set_position(glm::vec3{ 0, 0.5f + (i - 1), 0 });
		b.m_linear_momentum = { 0, -spd, 0 };
	}
	// Create pairs
	std::vector<overlap_pair> pair;
	for (size_t i = 1; i < bodies.size(); ++i)
	{
		body&     b = bodies[i];
		pair.push_back(overlap_pair{ &bodies[i - 1], &b, nullptr, nullptr });
		pair.back().manifold.normal = { 0,1,0 };
		pair.back().manifold.points.push_back(contact_point{ glm::vec3(0.0f, i == 1 ? 0.0 : 0.5, 0.0f), glm::vec3(0.0f, -0.5f, 0.0f) });
		pair.back().update();
	}
	//Fill overlap vector
	std::vector<overlap_pair *> pairs;
	for (int i = 0; i < pair.size(); ++i)
		pairs.push_back(&pair[i]);
	// Solve
	constraint_contact_solver{ 64, 0.0f }.evaluate(pairs);
	// Test solver
	for (size_t i = 0; i < pair.size(); ++i) {
		float supporting_mass = std::accumulate(bodies.begin() + i + 1, bodies.end(), 0.0f, [&](float acc, const body& body) { return acc + body.get_mass(); });
		ASSERT_NEAR(pair[i].manifold.points[0].lambda_Vel, supporting_mass * spd, 0.1f);
		body& b = *pair[i].body_B;
		ASSERT_NEAR(b.m_linear_momentum.y, 0.0f, 0.001f);
		ASSERT_NEAR(glm::length2(b.m_linear_momentum), 0.0f, 0.001f);
		ASSERT_NEAR(glm::length2(b.m_angular_momentum), 0.0f, 0.001f);
	}
}
TEST(constraint_solver, circle_single_restitution)
{
	// Define general speed
	float spd = 9.8f;
	// Create static body A
	body a;
	a.set_static(true);
	a.set_restitution(1.0f);
	// Create dynamic body B
	body b;
	b.set_mass(1.0f);
	b.set_inertia(glm::mat3{ 1.0f / 6.0f });
	b.set_position(glm::vec3{ 0, 0.5f, 0 });
	b.m_linear_momentum = { 0, -spd, 0 };
	b.set_restitution(1.0f);
	// Create pair
	overlap_pair pair{ &a, &b, nullptr, nullptr };
	pair.manifold.normal = { 0,1,0 };
	pair.manifold.points.push_back(contact_point{ glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -0.5f, 0.0f) });
	pair.update();
	//Fill overlap vector
	std::vector<overlap_pair *> pairs{ &pair };
	// Solve
	constraint_contact_solver{ 1, 0.0f }.evaluate(pairs);
	// Test solver
	ASSERT_NEAR(b.m_linear_momentum.y, spd, 0.0001f);
	ASSERT_NEAR(glm::length2(b.m_linear_momentum), spd*spd, 0.0001f);
	ASSERT_NEAR(glm::length2(b.m_angular_momentum), 0.0f, 0.0001f);
}
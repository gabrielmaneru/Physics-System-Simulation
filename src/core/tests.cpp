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
#include <physics/contact_solver.h>
TEST(naive_solver, simple_test)
{
	body a;
	a.m_linear_momentum = glm::vec3{ 0,-1,0 };
	a.set_position(glm::vec3{});
	a.set_mass(1.0f);
	a.set_inertia(glm::mat3{ 1.0f });

	body b;
	b.set_static(true);

	std::vector<contact_point> cts(1);
	contact_point& c{ cts[0] };
	c = { &a,&b };
	c.m_normal = glm::normalize(glm::vec3{ 1,1,0 });
	c.m_pi_A = c.m_pi_B = a.m_position - c.m_normal*5.0f;
	
	naive_contact_solver{}.evaluate(cts);
	ASSERT_NEAR(c.m_impulse, glm::sqrt(2.0f), 0.001f);
	
	ASSERT_TRUE(glm::all(glm::epsilonEqual(a.get_linear_velocity(), { 1, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(a.get_angular_velocity(), { 0, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(b.get_linear_velocity(), { 0, 0, 0 }, 0.001f)));
	ASSERT_TRUE(glm::all(glm::epsilonEqual(b.get_angular_velocity(), { 0, 0, 0 }, 0.001f)));
}
TEST(constraint_solver, circle_single)
{
	// Floor and 4 bodies
	std::vector<body> bodies(2);
	{
		// Floor (made static)
		bodies[0].set_static(true);

		// Circle (made dynamic)
		bodies[1].set_mass(1.0f);
		bodies[1].set_inertia(glm::mat3{ 1.0f / 6.0f });
		// Stack them (0.5, 1.5, 2.5, ...)
		bodies[1].set_position(glm::vec3{ 0, 0.5f, 0 });
		// Add them initial velocity (falling)
		bodies[1].m_linear_momentum = { 0, -9.8f, 0 };
	}

	// Contacts
	std::vector<contact_point> contacts;
	{
		contact_point c{ &bodies[0], &bodies[1] };
		// And the normal is facing upwards
		c.m_normal = { 0, 1, 0 };
		c.m_depth = 0.0f;
		// Collision point is just below the body
		c.m_pi_A = c.m_pi_B = bodies[1].m_position + glm::vec3(0, -0.5f, 0);
		contacts.push_back(c);
	}

	// Solve
	constraint_contact_solver{ 1, 0.0f, 0.0f }.evaluate(contacts);

	// Ensure correct impulse
	float supporting_mass = bodies[1].get_mass();
	ASSERT_NEAR(contacts[0].m_impulse, supporting_mass * 9.8f, 0.0001f);

	// Ensure velocity cancelation
	body& body = *contacts[0].m_body_B;
	
	ASSERT_NEAR(body.m_linear_momentum.y, 0.0f, 0.0001f);
	ASSERT_NEAR(glm::length2(body.m_linear_momentum), 0.0f, 0.0001f);
	ASSERT_NEAR(glm::length2(body.m_angular_momentum), 0.0f, 0.0001f);
}
#include <numeric>
TEST(constraint_solver, circle_stack)
{
	float spd = 10.0f * physics_dt;
	// Floor and 4 bodies
	std::vector<body> bodies(5);

	// Floor (made static)
	bodies[0].set_static(true);

	// Stacked bodies
	for (size_t i = 1; i < bodies.size(); ++i)
	{
		body& b = bodies[i];
		// Make dynamic
		b.set_mass(1.0f);
		b.set_inertia(glm::mat3{ 1.0f / 6.0f });
		// Stack them (0.5, 1.5, 2.5, ...)
		b.set_position(glm::vec3{ 0, 0.5f + (i - 1), 0 });
		// Add them initial velocity (falling)
		b.m_linear_momentum = { 0, -spd, 0 };
	}

	// Contacts
	std::vector<contact_point> contacts;
	for (size_t i = 1; i < bodies.size(); ++i)
	{
		body&     b = bodies[i];
		contact_point c{ &bodies[i - 1],&b };
		// And the normal is facing upwards
		c.m_normal = { 0, 1, 0 };
		c.m_depth = 0.0f;
		// Collision point is just below the body
		c.m_pi_A = c.m_pi_B = b.m_position + glm::vec3(0, -0.5f, 0);

		contacts.push_back(c);
	}

	// Solve
	constraint_contact_solver{ 64, 0.0f, 0.0f }.evaluate(contacts);

	// Ensure all bodies
	for (size_t i = 0; i < contacts.size(); ++i) {
		// Ensure correct impulse
		float supporting_mass = std::accumulate(bodies.begin() + i + 1, bodies.end(), 0.0f, [&](float acc, const body& body) { return acc + body.get_mass(); });
		ASSERT_NEAR(contacts[i].m_impulse, supporting_mass * spd, 0.1f);

		// Ensure velocity cancelation
		body& b = *contacts[i].m_body_B;
		ASSERT_NEAR(b.m_linear_momentum.y, 0.0f, 0.001f);
		ASSERT_NEAR(glm::length2(b.m_linear_momentum), 0.0f, 0.001f);
		ASSERT_NEAR(glm::length2(b.m_angular_momentum), 0.0f, 0.001f);
	}
}

TEST(constraint_solver, circle_single_restitution)
{
	// Floor and 4 bodies
	std::vector<body> bodies(2);
	{
		// Floor (made static)
		bodies[0].set_static(true);

		// Circle (made dynamic)
		bodies[1].set_mass(1.0f);
		bodies[1].set_inertia(glm::mat3{ 1.0f / 6.0f });
		// Stack them (0.5, 1.5, 2.5, ...)
		bodies[1].set_position(glm::vec3{ 0, 0.5f, 0 });
		// Add them initial velocity (falling)
		bodies[1].m_linear_momentum = { 0, -9.8f, 0 };
	}

	// Contacts
	std::vector<contact_point> contacts;
	{
		contact_point c{ &bodies[0], &bodies[1] };
		// And the normal is facing upwards
		c.m_normal = { 0, 1, 0 };
		c.m_depth = 0.0f;
		// Collision point is just below the body
		c.m_pi_A = c.m_pi_B = bodies[1].m_position + glm::vec3(0, -0.5f, 0);
		contacts.push_back(c);
	}

	// Solve
	constraint_contact_solver{ 1, 0.0f, 1.0f }.evaluate(contacts);

	// Ensure velocity cancelation
	body& body = *contacts[0].m_body_B;
	ASSERT_NEAR(body.m_linear_momentum.y, 9.8f, 0.0001f);
	ASSERT_NEAR(glm::length2(body.m_linear_momentum), 9.8f*9.8f, 0.0001f);
	ASSERT_NEAR(glm::length2(body.m_angular_momentum), 0.0f, 0.0001f);
}
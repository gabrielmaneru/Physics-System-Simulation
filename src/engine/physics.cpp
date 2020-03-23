/**
 * @file physics.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Physics system manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "physics.h"
#include "drawer.h"
#include <physics/gjk.h>
#include <physics/epa.h>
#include <physics/contact_solver.h>

/**
 * Perform ray instersection with the world
**/
ray_info_detailed c_physics::ray_cast(const ray & world_ray)const
{
	ray_info_detailed info;
	for (uint i = 0; i < m_bodies.size(); i++)
	{
		glm::mat4 model = m_bodies[i].get_model();
		glm::mat4 inv = glm::inverse(model);

		// Create ray
		ray local_ray = { tr_point(inv,world_ray.m_start),
			tr_vector(inv,world_ray.m_direction) };

		// Ray cast
		ray_info local_info = m_meshes[i].ray_cast(local_ray);

		if (local_info.m_intersected && local_info.m_time < info.m_time)
		{
			info.m_intersected = true;
			info.m_time = local_info.m_time;
			info.m_normal = glm::transpose(glm::inverse(glm::mat3(model)))*local_info.m_normal;

			info.m_pi = world_ray.get_point(info.m_time);
			info.m_body = i;
		}
	}
	return info;
}

contact c_physics::collision_narrow(const physical_mesh & m1,
	const physical_mesh & m2,
	body & b1,
	body & b2) const
{
	glm::vec3 init_dir = glm::normalize(b2.m_position - b1.m_position);

	// Solve using GJK Algorithm
	gjk gjk_solver(m1, m2,
		b1.get_model(), b2.get_model(),
		b1.get_basis(), b2.get_basis());

	// Call GJK solver
	gjk_solver.evaluate(init_dir);

	// Draw Minkowski
	if (m_draw_minkowski)
		for (auto v1 : m1.m_vertices)
			for (auto v2 : m2.m_vertices)
				drawer.add_debugline_cube(v1 - tr_point(gjk_solver.m_mod_to_A, v2), 0.1f, black);

	// Draw GJK Simplex
	if (m_draw_gjk_simplex)
		for (uint i = 0; i < gjk_solver.m_simplex.m_dim; ++i)
			for (uint j = 0; j < gjk_solver.m_simplex.m_dim; ++j)
				drawer.add_debugline(gjk_solver.m_simplex.m_points[i],
					gjk_solver.m_simplex.m_points[j], white);

	// If Solver success -> Origin is encloseed by simplex
	if (gjk_solver.m_status == gjk::e_Success)
	{


		// Create EPA solver
		epa epa_solver{ gjk_solver };

		// Draw EPA Simplex
		if (m_draw_epa_simplex)
		{
			drawer.add_debugline_list(epa_solver.m_polytope.get_lines(), blue);
			drawer.add_debugtri_list(epa_solver.m_polytope.get_triangles(), blue);
		}

		// Call EPA solver
		epa_solver.evaluate();


		// Draw EPA Polytope
		if (m_draw_epa_polytope)
		{
			glm::vec3 color = epa_solver.m_status == epa::e_Success ? green : red;
			drawer.add_debugline_list(epa_solver.m_polytope.get_lines(), color);
			drawer.add_debugtri_list(epa_solver.m_polytope.get_triangles(), color);
			if(epa_solver.m_status == epa::e_Success)
				drawer.add_debugtri_list({
					epa_solver.m_result.m_points[0],
					epa_solver.m_result.m_points[1],
					epa_solver.m_result.m_points[2]
					}, red);
		}

		// If Success
		if (epa_solver.m_status == epa::e_Success)
		{
			// Fill contact info
			contact result{&b1,&b2};

			glm::vec3 p0A = tr_point(b1.get_model(), gjk_solver.supportA(epa_solver.m_result.m_dirs[0], m1));
			glm::vec3 p1A = tr_point(b1.get_model(), gjk_solver.supportA(epa_solver.m_result.m_dirs[1], m1));
			glm::vec3 p2A = tr_point(b1.get_model(), gjk_solver.supportA(epa_solver.m_result.m_dirs[2], m1));
			glm::vec3 p0B = tr_point(b1.get_model(), gjk_solver.supportB(-epa_solver.m_result.m_dirs[0], m2));
			glm::vec3 p1B = tr_point(b1.get_model(), gjk_solver.supportB(-epa_solver.m_result.m_dirs[1], m2));
			glm::vec3 p2B = tr_point(b1.get_model(), gjk_solver.supportB(-epa_solver.m_result.m_dirs[2], m2));

			result.m_pi_A = p0A * epa_solver.m_result.m_bary[0]
				+ p1A * epa_solver.m_result.m_bary[1]
				+ p2A * epa_solver.m_result.m_bary[2];
			result.m_pi_B = p0B * epa_solver.m_result.m_bary[0]
				+ p1B * epa_solver.m_result.m_bary[1]
				+ p2B * epa_solver.m_result.m_bary[2];

			result.m_depth = epa_solver.m_depth;
			result.m_normal = -epa_solver.m_normal;


			if (m_draw_epa_results)
			{
				// Draw Contacts
				drawer.add_debugline_cube(result.m_pi_A, 0.1f, red);
				drawer.add_debugline_cube(result.m_pi_B, 0.2f, blue);

				// Draw Normal line
				drawer.add_debugline(result.m_pi_A, result.m_pi_A + result.m_normal * result.m_depth, yellow);
			}
			return result;
		}
	}
	return {};
}

/**
 * Update Manager
**/
void c_physics::update()
{
	std::vector<contact> contacts;

	const int it_count{ 7 };
	const float time_step{ 1.0f / 60.0f };
	const float time_step_it = time_step / it_count;

	for (uint it = 0; it < it_count; ++it)
	{
		// Detect Collision
		for (uint i = 0; i < m_bodies.size() - 1; ++i)
		{
			body& b1 = m_bodies[i];
			const physical_mesh& m1 = m_meshes[i];
			for (uint j = i + 1; j < m_bodies.size(); ++j)
			{
				body& b2 = m_bodies[j];
				const physical_mesh& m2 = m_meshes[j];

				// Collide the two meshes
				contact result = collision_narrow(m1, m2, b1, b2);
				if (result.m_hit)
					contacts.emplace_back(std::move(result));
			}
		}

		// Solve Contacts
		naive_contact_solver{}.evaluate(contacts);
		contacts.clear();

		// Integrate bodies
		for (auto& b : m_bodies)
			b.integrate(time_step_it);
	}
}

/**
 * Clean bodies
**/
void c_physics::clean()
{
	m_bodies.clear();
	m_meshes.clear();
}

/**
 *  Add a body to the system
**/
body& c_physics::add_body(std::string file)
{
	auto it = m_loaded_meshes.find(file);
	if (it == m_loaded_meshes.end())
	{
		const char * path = "../resources/meshes/";
		raw_mesh mesh{ path + file };
		m_loaded_meshes[file] = mesh;
		it = m_loaded_meshes.find(file);
	}
	raw_mesh& raw = it->second;

	physical_mesh m;
	m.m_vertices = { raw.m_vertices.begin(), raw.m_vertices.end() };
	for (auto f : raw.m_faces)
		m.add_face(f);
	m.create_twins();
	m.merge_coplanar();

	m_meshes.emplace_back(std::move(m));
	m_bodies.push_back({});
	m_bodies.back().set_mass(raw.m_mass).set_inertia(raw.m_inertia);
	return m_bodies.back();
}

/**
 *  Singletone instanciation
**/
c_physics & c_physics::get_instance()
{
	static c_physics instance;
	return instance;
}

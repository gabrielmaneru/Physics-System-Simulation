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

contact_info c_physics::collision_narrow(const physical_mesh & m1,
	const physical_mesh & m2,
	const body & b1,
	const body & b2) const
{
	glm::vec3 init_dir = glm::normalize(b2.m_position - b1.m_position);

	// Solve using GJK Algorithm
	gjk gjk_solver(m1, m2,
		b1.get_model(), b2.get_model(),
		b1.get_basis(), b2.get_basis());

	// Call GJK solver
	gjk_solver.evaluate(init_dir);

	// If Solver success -> Origin is encloseed by simplex
	if (gjk_solver.m_status == gjk::e_Success)
	{
		// Draw Minkowski
		if(m_draw_minkowski)
			for (auto v1 : m1.m_vertices)
				for (auto v2 : m2.m_vertices)
					drawer.add_debugline_cube(v1 - tr_point(gjk_solver.m_mod_to_A, v2), 0.1f, black);

		// Draw GJK Simplex
		if (m_draw_gjk_simplex)
			for (uint i = 0; i < gjk_solver.m_simplex.m_dim; ++i)
				for(uint j=0; j < gjk_solver.m_simplex.m_dim; ++j)
					drawer.add_debugline(gjk_solver.m_simplex.m_points[i],
						gjk_solver.m_simplex.m_points[j], white);


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
			contact_info result{ true };

			result.m_points[0] = tr_point(b1.get_model(), gjk_solver.supportA(epa_solver.m_result.m_dirs[0], m1));
			result.m_points[1] = tr_point(b1.get_model(), gjk_solver.supportA(epa_solver.m_result.m_dirs[1], m1));
			result.m_points[2] = tr_point(b1.get_model(), gjk_solver.supportA(epa_solver.m_result.m_dirs[2], m1));
			result.m_points[3] = tr_point(b1.get_model(), gjk_solver.supportB(-epa_solver.m_result.m_dirs[0], m2));
			result.m_points[4] = tr_point(b1.get_model(), gjk_solver.supportB(-epa_solver.m_result.m_dirs[1], m2));
			result.m_points[5] = tr_point(b1.get_model(), gjk_solver.supportB(-epa_solver.m_result.m_dirs[2], m2));

			result.m_bary = epa_solver.m_result.m_bary;

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
	for (uint i = 0; i < m_bodies.size() - 1; ++i)
	{
		const body& b1 = m_bodies[i];
		const physical_mesh& m1 = m_meshes[i];
		for (uint j = i + 1; j < m_bodies.size(); ++j)
		{
			const body& b2 = m_bodies[j];
			const physical_mesh& m2 = m_meshes[j];

			// Collide the two meshes
			contact_info result = collision_narrow(m1, m2, b1, b2);

			// If valid contact info
			if (result.m_hit && m_draw_epa_results)
			{
				// Draw contact points
				drawer.add_debugline_cube(result.m_points[0], 0.1f, green);
				drawer.add_debugline_cube(result.m_points[1], 0.1f, green);
				drawer.add_debugline_cube(result.m_points[2], 0.1f, green);
				drawer.add_debugline_cube(result.m_points[3], 0.1f, red);
				drawer.add_debugline_cube(result.m_points[4], 0.1f, red);
				drawer.add_debugline_cube(result.m_points[5], 0.1f, red);

				// Draw Point of intersection in A
				glm::vec3 av0 = result.m_points[0] * result.m_bary[0]
							+   result.m_points[1] * result.m_bary[1]
							+   result.m_points[2] * result.m_bary[2];
				drawer.add_debugline_cube(av0, 0.1f, black);
				drawer.add_debugline_cube(av0, 0.2f, green);

				// Draw Point of intersection in B
				glm::vec3 av1 = result.m_points[3] * result.m_bary[0]
							+   result.m_points[4] * result.m_bary[1]
							+   result.m_points[5] * result.m_bary[2];
				drawer.add_debugline_cube(av1, 0.1f, black);
				drawer.add_debugline_cube(av1, 0.2f, red);

				// Draw Normal line
				drawer.add_debugline(av0, av1, yellow);
			}
		}
	}

	// Integrate bodies
	for (auto& b : m_bodies)
		b.integrate(1.0f/60.0f);
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
	m_bodies.back().set
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

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
		ray local_ray = { tr_point(inv,world_ray.m_start),
			tr_vector(inv,world_ray.m_direction) };
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

bool c_physics::collision_narrow(const physical_mesh & m1,
	const physical_mesh & m2,
	const body & b1,
	const body & b2) const
{
	glm::vec3 init_dir = glm::normalize(b2.m_position - b1.m_position);

	// Solve using GJK Algorithm
	gjk gjk_solver(m1, m2, b1.get_model(), b2.get_model());
	gjk_solver.evaluate(-init_dir);

	// If Solver success -> Origin is encloseed by simplex
	if (gjk_solver.m_status == gjk::e_Success)
	{
		// minkow
		for (auto v1 : m1.m_vertices)
			for (auto v2 : m2.m_vertices)
				drawer.add_debugline_cube(tr_point(gjk_solver.m_mod_A, v1) - tr_point(gjk_solver.m_mod_B, v2), 0.1f, black);
		
		epa epa_solver{ gjk_solver };
		drawer.add_debugtri_list(epa_solver.m_polytope.get_triangles(), blue);
		drawer.add_debugline_list(epa_solver.m_polytope.get_lines(), blue);

		epa_solver.evaluate();
		if (epa_solver.m_status == epa::e_Success)
		{
			drawer.add_debugline_list(epa_solver.m_polytope.get_lines(), green);
			drawer.add_debugtri_list(epa_solver.m_polytope.get_triangles(), green);

			glm::vec3 w{ 0.f };
			for (uint i = 0; i < 3; ++i)
				w += gjk_solver.support(epa_solver.m_result.m_dirs[i])*epa_solver.m_result.m_bary[i];

			drawer.add_debugline_cube(w, 0.1f, green);
			drawer.add_debugline(w, glm::vec3{ 0.0f }, green*3.0f);

			return true;
		}
		else
		{
			drawer.add_debugline_list(epa_solver.m_polytope.get_lines(), red);
			drawer.add_debugtri_list(epa_solver.m_polytope.get_triangles(), red);
		}

	}
	return false;
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
			collision_narrow(m1, m2, b1, b2);
		}
	}
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

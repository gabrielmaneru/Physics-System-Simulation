/**
 * @file physics.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Physics system manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "physics.h"
#include "drawer.h"
#include "window.h"
#include "editor.h"
#include <physics/sat.h>
#include <physics/contact_solver.h>
#include <physics/math_utils.h>

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
/**
 * Perform carrow collision detection of the pair
**/
bool c_physics::collision_narrow(overlap_pair * pair) const
{
	// Initialize algorithm
	sat algorithm{ pair };
	// Run algorithm
	sat::result r = algorithm.test_collision();
	// If contact found
	if (r.m_contact)
	{
		// Add new manifold data
		pair->add_manifold(r.m_manifold);
		// Set Collision state
		pair->m_state = overlap_pair::state::Collision;
		return true;
	}
	// If no penetration found
	else
	{
		// Remove old manifold data
		pair->manifold.points.clear();
		// Set Collision state
		pair->m_state = overlap_pair::state::NoCollision;
		return false;
	}
}

/**
 * Update Manager
**/
void c_physics::update()
{
	// Update physics delta time
	physics_dt = static_cast<float>(window.m_dt);
	// Current contact information
	std::vector<overlap_pair*> contacts;
	// Integrate velocities
	for (auto& b : m_bodies)
		b.integrate_velocities(physics_dt, m_gravity);
	// Detect collision
	for (uint i = 0; i < m_bodies.size() - 1; ++i)
		for (uint j = i + 1; j < m_bodies.size(); ++j)
		{
			// Get mutual pair
			overlap_pair* pair = &m_overlaps[{i, j}];
			// If new pair, initialize it properly
			if (pair->m_state == overlap_pair::state::New)
				*pair = { &m_bodies[i],&m_bodies[j],&m_meshes[i],&m_meshes[j] };
			// Perform narrow collision detection
			if (collision_narrow(pair))
			{
				// Update pair information
				pair->update();
				contacts.push_back(pair);
			}
		}
	// Solve velocity Contraints
	constraint_contact_solver{editor.m_solver_iterations, editor.m_baumgarte, editor.m_do_warm_start}.evaluate(contacts);
	// Draw debug contact points
	for (auto o : contacts)
	for (auto p : o->manifold.points)
	{
		const glm::vec3 pA = tr_point(o->body_A->get_model(), p.local_A);
		const glm::vec3 pB = tr_point(o->body_B->get_model(), p.local_B);
		drawer.add_debugline(pA, pB, red);
		drawer.add_debugline_cube(pA, 0.1f, red);
		drawer.add_debugline_cube(pB, 0.1f, blue);
		drawer.add_debugline(pA, pA + o->manifold.normal * p.depth, green);
	}
	// Integrate positions
	for (auto& b : m_bodies)
		b.integrate_positions(physics_dt);
}

/**
 * Clean bodies
**/
void c_physics::clean()
{
	m_bodies.clear();
	m_meshes.clear();
	m_overlaps.clear();
}

/**
 *  Add a body to the system
**/
body& c_physics::add_body(std::string file)
{
	// Find the mesh in loaded raw meshes
	auto it = m_loaded_meshes.find(file);
	// If not found, load it
	if (it == m_loaded_meshes.end())
	{
		// Meshes path
		const char * path = "../resources/meshes/";
		//Load raw mesh
		raw_mesh mesh{ path + file };
		// Add it to the map
		m_loaded_meshes[file] = mesh;
		// Refresh iterator
		it = m_loaded_meshes.find(file);
	}
	// Get raw mesh
	raw_mesh& raw = it->second;

	// Create physical mesh
	physical_mesh m;
	// Copy vertex array
	m.m_vertices = { raw.m_vertices.begin(), raw.m_vertices.end() };
	// Insert triangles
	for (auto f : raw.m_triangles)
		m.add_face(f);
	// Connect twins
	m.create_twins();
	// Merge coplanar faces
	m.merge_coplanar();
	// Move mesh into the final array
	m_meshes.emplace_back(std::move(m));
	// Create new body
	m_bodies.push_back({});
	// Initialize with mesh properties
	m_bodies.back().set_mass(raw.m_mass).set_inertia(raw.m_inertia);
	// Return newly created body
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

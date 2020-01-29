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
#include "input.h"
#include <imgui/imgui.h>

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

/**
 * Add the debug lines of the bodies
**/
void c_physics::draw_debug_bodies() const
{
	for (uint i = 0; i < m_bodies.size(); i++)
	{
		const physical_mesh& mesh = m_meshes[i];
		glm::mat4 m = m_bodies[i].get_model();
		glm::vec3 color = ((uint)m_hovered == i) ? magenta : red;
		for (auto f : mesh.m_faces)
		{
			half_edge* hedge = f.m_hedge_start;
			do
			{
				glm::vec3 p0 = tr_point(m, mesh.m_vertices[hedge->get_start()]);
				glm::vec3 p1 = tr_point(m, mesh.m_vertices[hedge->get_end()]);
				drawer.add_debug_line(p0, p1, color);
				hedge = hedge->m_next;
			} while (hedge != f.m_hedge_start);
		}
		drawer.add_debug_line(m_bodies[i].m_position, m_bodies[i].m_position + m_bodies[i].m_linear_momentum, green);
		drawer.add_debug_line(m_bodies[i].m_position, m_bodies[i].m_position + m_bodies[i].m_angular_momentum, blue);
	}
}

/**
 * Manage object picking and hovering
**/
void c_physics::do_object_picking()
{
	glm::vec4 mouse_ndc{ glm::vec3{window.get_mouse_ndc(),0.0f},1.0f };
	glm::vec4 mouse_world = glm::inverse(drawer.m_camera.get_vp())*mouse_ndc;
	glm::vec3 mouse_pos{ glm::vec3(mouse_world) / mouse_world.w };
	ray mouse_ray{ mouse_pos, mouse_pos - drawer.m_camera.m_eye };

	ray_info_detailed info = ray_cast(mouse_ray);
	if (info.m_intersected)
	{
		body& b = m_bodies[info.m_body];
		drawer.add_debug_line(info.m_pi, b.m_position, yellow);
		drawer.add_debug_line(info.m_pi, info.m_pi + 0.1f*info.m_normal, magenta);
		drawer.add_debug_cube(mouse_ray.m_start, 0.0001f, white);
		m_hovered = static_cast<int>(info.m_body);

		if (input.m_mouse_triggered[0])
		{
			if (m_selected == m_hovered)
			{
				glm::vec3 force = glm::normalize(mouse_ray.m_direction);
				if(input.is_key_down(GLFW_KEY_LEFT_SHIFT))
					force*=0.01f;
				b.add_force(force, info.m_pi);
			}
			else
				m_selected = m_hovered;
		}
	}
	else
	{
		m_hovered = -1;
		if (input.m_mouse_triggered[0])
			m_selected = -1;
	}
}

/**
 * Initialize Manager
**/
bool c_physics::initialize()
{
	add_body("cube.obj")
		.set_position({ 0.0f, 0.0f, 0.0f })
		.set_inertia({ 1.f / 6.f, 0.f, 0.f,
					   0.f, 1.f / 6.f, 0.f,
					   0.f, 0.f, 1.f / 6.f });

	add_body("sphere.obj")
		.set_position({ 3.0f, 0.0f, 0.0f })
		.set_inertia({ 2.f / 5.f, 0.f, 0.f,
					   0.f, 2.f / 5.f, 0.f,
					   0.f, 0.f, 2.f / 5.f });

	add_body("cylinder.obj")
		.set_position({ 4.0f, 0.0f, 0.0f })
		.set_inertia({ 1.f / 2.f, 0.f, 0.f,
					   0.f, 1.f / 4.f, 0.f,
					   0.f, 0.f, 1.f / 2.f });

	add_body("gourd.obj")
		.set_position({ 0.0f, 0.0f, 2.0f });

	add_body("icosahedron.obj")
		.set_position({ 0.0f, 0.0f, -2.0f });

	add_body("octohedron.obj")
		.set_position({ 2.0f, 0.0f, 2.0f });

	add_body("quad.obj")
		.set_position({ 4.0f, 0.0f, 2.0f })
		.set_inertia({ 1.f / 2.f, 0.f, 0.f,
					   0.f, 1.f / 2.f, 0.f,
					   0.f, 0.f, 1.f / 4.f });

	add_body("triangle.obj")
		.set_position({ 2.0f, 0.0f, 4.0f })
		.set_inertia({ 1.f / 2.f, 0.f, 0.f,
					   0.f, 1.f / 2.f, 0.f,
					   0.f, 0.f, 1.f / 4.f });
	return true;
}

/**
 * Update Manager
**/
void c_physics::update()
{
	for (auto& b : m_bodies)
		b.integrate(1.0f/60.0f);

	if(!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
		do_object_picking();
	draw_debug_bodies();
	if (input.is_key_triggered(GLFW_KEY_R))
		reset();
}

/**
 * Shutdown Manager
**/
void c_physics::shutdown()
{
	m_bodies.clear();
	m_meshes.clear();
}

/**
 *  Reset Manager
**/
void c_physics::reset()
{
	shutdown();
	initialize();
	m_hovered = -1;
	m_selected = -1;
}

/**
 *  Draw the body properties if selected
**/
void c_physics::drawGUI()
{
	if (m_selected >= 0)
	{
		body& b = m_bodies[m_selected];
		if (ImGui::Begin("Body", nullptr))
		{
			ImGui::InputFloat3("Position", &b.m_position.x);
			ImGui::InputFloat4("Rotation", &b.m_rotation.x);
			ImGui::InputFloat3("Linear M", &b.m_position.x);
			ImGui::InputFloat3("Angular M", &b.m_position.x);
			ImGui::InputFloat("Mass", &b.m_mass);

			ImGui::NewLine();
			ImGui::NewLine();

			glm::mat3 i = glm::inverse(b.m_inv_inertia);
			ImGui::InputFloat3("Inertia", &i[0].x);
			ImGui::InputFloat3("", &i[1].x);
			ImGui::InputFloat3("", &i[2].x);
			ImGui::End();
		}
	}
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

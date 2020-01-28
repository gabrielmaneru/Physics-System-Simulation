#include "physics.h"
#include "drawer.h"
#include "window.h"
#include "input.h"
#include <iostream>

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
void c_physics::draw_debug_bodies() const
{
	for (uint i = 0; i < m_bodies.size(); i++)
	{
		const physical_mesh& mesh = m_meshes[i];
		glm::mat4 m = m_bodies[i].get_model();
		for (auto f : mesh.m_faces)
		{
			half_edge* hedge = f.m_hedge_start;
			do
			{
				glm::vec3 p0 = tr_point(m, mesh.m_vertices[hedge->get_start()]);
				glm::vec3 p1 = tr_point(m, mesh.m_vertices[hedge->get_end()]);
				drawer.add_debug_line(p0, p1, red);
				hedge = hedge->m_next;
			} while (hedge != f.m_hedge_start);
		}
		drawer.add_debug_line(m_bodies[i].m_position, m_bodies[i].m_position + m_bodies[i].m_linear_momentum, green);
		drawer.add_debug_line(m_bodies[i].m_position, m_bodies[i].m_position + m_bodies[i].m_angular_momentum, blue);
	}
}
ray c_physics::get_mouse_ray() const
{
	glm::vec4 mouse_ndc{ glm::vec3{window.get_mouse_ndc(),0.0f},1.0f };
	glm::vec4 mouse_world = glm::inverse(drawer.m_camera.get_vp())*mouse_ndc;
	glm::vec3 mouse_pos{ glm::vec3(mouse_world) / mouse_world.w };

	return{ mouse_pos, mouse_pos - drawer.m_camera.m_eye };
}

bool c_physics::initialize()
{
	body b{};
	b.m_position = glm::vec3(0.0f, 0.0f, 0.0f);
	add_body(b, "cube.obj");

	b.m_position = glm::vec3(2.0f, 0.0f, 0.0f);
	add_body(b, "sphere.obj");

	b.m_position = glm::vec3(-2.0f, 0.0f, 0.0f);
	add_body(b, "cylinder.obj");

	b.m_position = glm::vec3(0.0f, 0.0f, 2.0f);
	add_body(b, "gourd.obj");

	b.m_position = glm::vec3(0.0f, 0.0f, -2.0f);
	add_body(b, "icosahedron.obj");

	b.m_position = glm::vec3(2.0f, 0.0f, 2.0f);
	add_body(b, "octohedron.obj");

	b.m_position = glm::vec3(4.0f, 0.0f, 2.0f);
	add_body(b, "quad.obj");

	b.m_position = glm::vec3(2.0f, 0.0f, 4.0f);
	add_body(b, "triangle.obj");

	return true;
}
void c_physics::update()
{
	for (auto& b : m_bodies)
		b.integrate(1.0f/60.0f);
	draw_debug_bodies();

	ray mouse_ray = get_mouse_ray();

	ray_info_detailed info = ray_cast(mouse_ray);
	if (info.m_intersected)
	{
		body& b = m_bodies[info.m_body];
		drawer.add_debug_line(info.m_pi, tr_point(b.get_model(),b.m_mass_center), yellow);
		drawer.add_debug_line(info.m_pi, info.m_pi + 0.1f*info.m_normal, magenta);
		drawer.add_debug_cube(mouse_ray.m_start, 0.0001f, white);
		if (input.m_mouse_triggered[0])
		{
			b.add_force(glm::normalize(mouse_ray.m_direction), info.m_pi);
		}
	}
}
void c_physics::shutdown()
{
}

body& c_physics::add_body(const body & b, std::string file)
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

	m_bodies.push_back(b);
	m_meshes.emplace_back(std::move(m));
	return m_bodies.back();
}
c_physics & c_physics::get_instance()
{
	static c_physics instance;
	return instance;
}

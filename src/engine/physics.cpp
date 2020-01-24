#include "physics.h"
#include "drawer.h"
#include "window.h"
#include <iostream>

inline glm::vec3 tr_point(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 1));
}
inline glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 0));
}

float c_physics::ray_cast(const ray & world_ray)const
{
	float min_time{ FLT_MAX };
	for (uint i = 0; i < m_bodies.size(); i++)
	{
		glm::mat4 inv = glm::inverse(m_bodies[i].get_model());
		ray local_ray = { tr_point(inv,world_ray.m_start),
			tr_vector(inv,world_ray.m_direction) };
		float local_time = m_meshes[i].ray_cast(local_ray);
		if (local_time >= 0 && local_time < min_time)
			min_time = local_time;
	}
	return (min_time == FLT_MAX) ? -1.0f : min_time;
}

void c_physics::add_debug_lines() const
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
				drawer.add_debug_line(p0, p1, glm::vec3(1, 0, 0));
				hedge = hedge->m_next;
			} while (hedge != f.m_hedge_start);
		}
	}
}

bool c_physics::initialize()
{
	body b{};
	b.m_position = glm::vec3(-1.0f, 0.0f, 0.0f);
	add_body(b, "cube.obj");
	b.m_position = glm::vec3(1.0f, 0.0f, 0.0f);
	add_body(b, "cube.obj");
	return true;
}

void c_physics::update()
{
	add_debug_lines();
	glm::vec4 mouse_ndc{ glm::vec3{window.get_mouse_ndc(),1.0f},1.0f };
	glm::vec4 mouse_world = glm::inverse(drawer.m_camera.get_vp())*mouse_ndc;
	glm::vec3 mouse_pos{ glm::vec3(mouse_world) / mouse_world.w };

	glm::vec3 cam_eye = drawer.m_camera.m_eye;
	ray cam_mouse = { cam_eye, mouse_pos - cam_eye };
	float time = ray_cast(cam_mouse);
	if(time < 0.0f)
		drawer.add_debug_cube(mouse_pos, 5.f, glm::vec3{0.0f, 0.0f, 1.0f});
	else
		drawer.add_debug_cube(mouse_pos, 5.f, glm::vec3{ 0.0f, 1.0f, 0.0f });

}

void c_physics::shutdown()
{
}

void c_physics::add_body(const body & b, std::string file)
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
}

c_physics & c_physics::get_instance()
{
	static c_physics instance;
	return instance;
}

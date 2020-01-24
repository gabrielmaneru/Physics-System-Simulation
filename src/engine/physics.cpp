#include "physics.h"
#include "drawer.h"

bool c_physics::initialize()
{
	body b{};
	b.m_position = glm::vec3(-1.0f, 0.0f, 0.0f);
	add_body(b, "bunny.obj");
	//b.m_position = glm::vec3(1.0f, 0.0f, 0.0f);
	//add_body(b, "cube.obj");
	return true;
}

void c_physics::update()
{
	for (uint i = 0; i < m_bodies.size(); i++)
	{
		physical_mesh& mesh = m_meshes[i];
		glm::mat4 m = m_bodies[i].get_model();
		for (auto f : mesh.m_faces)
		{
			half_edge* hedge = f.m_hedge_start;
			do
			{
				glm::vec3 p0 = glm::vec3(m * glm::vec4(mesh.m_vertices[hedge->get_start()].m_position, 1.0f));
				glm::vec3 p1 = glm::vec3(m * glm::vec4(mesh.m_vertices[hedge->get_end()].m_position,1.0f));
				drawer.add_debug_line(p0, p1, glm::vec3(1, 0, 0));
				hedge = hedge->m_next;
			} while (hedge != f.m_hedge_start);
		}
	}
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

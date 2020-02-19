/**
 * @file editor.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief GUI Manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "editor.h"
#include "physics.h"
#include "drawer.h"
#include "input.h"
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include "window.h"

/**
 * ImGui system
**/
bool c_editor::ImGui_Init()const
{
	ImGui::CreateContext();
	if (!ImGui_ImplGlfw_InitForOpenGL(window.get_handler(), true))
		return false;
	const char * glsl_v = "#version 330";
	if (!ImGui_ImplOpenGL3_Init(glsl_v))
		return false;

	ImGui::StyleColorsLight();
	ImGui::GetStyle().FrameRounding = 12;
	ImGui::GetStyle().Colors[2] = ImVec4{ 1.0f, 1.0f, 1.0f, 0.6f };
	ImGui::GetStyle().Colors[10] = ImVec4{ 0.8f, 0.0f, 0.2f, 1.0f };
	ImGui::GetStyle().Colors[11] = ImVec4{ 0.8f, 0.0f, 0.2f, 1.0f };
	return true;
}
void c_editor::ImGui_Shutdown()const
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

/**
 * Add Objects to the scene
**/
void c_editor::create_scene() const
{
	physics.add_body("cube.obj")
		.set_position({ 0.0f, 0.0f, 0.0f })
		.set_inertia({ 1.f / 6.f, 0.f, 0.f,
					   0.f, 1.f / 6.f, 0.f,
					   0.f, 0.f, 1.f / 6.f });

	physics.add_body("sphere.obj")
		.set_position({ 3.0f, 0.0f, 0.0f })
		.set_inertia({ 2.f / 5.f, 0.f, 0.f,
					   0.f, 2.f / 5.f, 0.f,
					   0.f, 0.f, 2.f / 5.f });

	/*physics.add_body("cylinder.obj")
		.set_position({ 0.0f, 0.0f, 3.0f })
		.set_inertia({ 1.f / 2.f, 0.f, 0.f,
					   0.f, 1.f / 4.f, 0.f,
					   0.f, 0.f, 1.f / 2.f });

	physics.add_body("gourd.obj")
		.set_position({ 3.0f, 0.0f, 3.0f });

	physics.add_body("icosahedron.obj")
		.set_position({ 6.0f, 0.0f, 3.0f });

	physics.add_body("octohedron.obj")
		.set_position({ 3.0f, 0.0f, 6.0f });

	physics.add_body("quad.obj")
		.set_position({ 6.0f, 0.0f, 6.0f })
		.set_inertia({ 1.f / 2.f, 0.f, 0.f,
					   0.f, 1.f / 2.f, 0.f,
					   0.f, 0.f, 1.f / 4.f });

	physics.add_body("triangle.obj")
		.set_position({ 9.0f, 0.0f, 6.0f })
		.set_inertia({ 1.f / 2.f, 0.f, 0.f,
					   0.f, 1.f / 2.f, 0.f,
					   0.f, 0.f, 1.f / 4.f });

	physics.add_body("bunny.obj")
		.set_position({ 6.0f, 0.0f, 9.0f });*/
}

void c_editor::reset_scene()
{
	physics.clean();
	create_scene();
	m_hovered = -1;
	m_selected = -1;
}

/**
 * Add the debug lines of the bodies
**/
void c_editor::draw_debug_bodies()const
{
	for (uint i = 0; i < physics.m_bodies.size(); i++)
	{
		const physical_mesh& mesh = physics.m_meshes[i];
		std::vector<glm::vec3> lines = mesh.get_lines();
		std::vector<glm::vec3> tri = mesh.get_triangles();

		const body& bdy = physics.m_bodies[i];
		glm::mat4 m = bdy.get_model();
		for (auto& p : lines)
			p = tr_point(m, p);
		for (auto& p : tri)
			p = tr_point(m, p);

		glm::vec3 color = ((uint)m_hovered == i) ? magenta : red;
		drawer.add_debugline_list(lines, color);
		drawer.add_debugtri_list(tri, white);

		drawer.add_debugline(bdy.m_position, bdy.m_position + bdy.m_linear_momentum, green);
		drawer.add_debugline(bdy.m_position, bdy.m_position + bdy.m_angular_momentum, blue);
	}
}

/**
 * Manage object picking and hovering
**/
void c_editor::object_picking()
{
	glm::vec4 mouse_ndc{ glm::vec3{window.get_mouse_ndc(),0.0f},1.0f };
	glm::vec4 mouse_world = glm::inverse(drawer.m_camera.get_vp())*mouse_ndc;
	glm::vec3 mouse_pos{ glm::vec3(mouse_world) / mouse_world.w };
	ray mouse_ray{ mouse_pos, glm::normalize(mouse_pos - drawer.m_camera.m_eye) };

	ray_info_detailed info = physics.ray_cast(mouse_ray);
	if (info.m_intersected)
	{
		body& b = physics.m_bodies[info.m_body];
		drawer.add_debugline(info.m_pi, b.m_position, yellow);
		drawer.add_debugline(info.m_pi, info.m_pi + 0.1f*info.m_normal, magenta);
		drawer.add_debugline_cube(mouse_ray.m_start, 0.0001f, white);
		{
			glm::mat4 m = b.get_model();
			glm::mat4 inv_m = glm::inverse(m);
			glm::vec3 dir = glm::normalize(tr_vector(inv_m, info.m_pi - b.m_position));

			glm::vec3 sp1 = physics.m_meshes[info.m_body].support_point_bruteforce(dir);
			glm::vec3 sp2 = physics.m_meshes[info.m_body].support_point_hillclimb(dir);

			sp1 = tr_point(m, sp1);
			sp2 = tr_point(m, sp2);

			drawer.add_debugline_cube(sp1, 0.08f, green);
			drawer.add_debugline_cube(sp2, 0.06f, red);
		}
		m_hovered = static_cast<int>(info.m_body);

		if (input.m_mouse_triggered[0])
		{
			if (m_selected == m_hovered)
			{
				glm::vec3 force = glm::normalize(mouse_ray.m_direction);
				if (input.is_key_down(GLFW_KEY_LEFT_SHIFT))
					force *= 0.01f;
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

bool c_editor::initialize()
{
	if (!ImGui_Init())
		return false;

	create_scene();
	return true;
}
void c_editor::update()
{
	draw_debug_bodies();
	if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
		object_picking();
	if (input.is_key_triggered(GLFW_KEY_R))
		reset_scene();
}
void c_editor::drawGui()const
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	if (m_selected >= 0)
	{
		body& b = physics.m_bodies[m_selected];
		if (ImGui::Begin("Body", nullptr))
		{
			ImGui::DragFloat3("Position", &b.m_position.x, 0.01f);
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

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
void c_editor::shutdown()
{
	ImGui_Shutdown();
}
c_editor & c_editor::get_instance()
{
	static c_editor instance;
	return instance;
}

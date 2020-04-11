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
#include <imgui/ImGuizmo.h>
#include "window.h"
#include <physics/epa.h>
#include <physics/math_utils.h>

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
	ImGuizmo::SetOrthographic(false);
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
		.set_position({ 0.0f, -20.0f, 0.0f })
		.set_static(true);
	physics.m_meshes.back().scale(40.f);

	//physics.add_body("sphere.obj")
	//	.set_position({ -1.0f, 0.5f, 0.0f })
	//	.set_mass(1.0f);

	physics.add_body("cube.obj")
		.set_position({ 1.1f, 0.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 0.0f, 0.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ -1.1f, 0.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 1.1f, 1.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 0.0f, 1.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ -1.1f, 1.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 1.1f, 2.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 0.0f, 2.5f, 0.0f })
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ -1.1f, 2.5f, 0.0f })
		.set_mass(1.0f);
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
	float size = 20.0f;
	drawer.add_debugline_cube(glm::vec3(0.0f), 0.1f, white);
	drawer.add_debugline(glm::vec3(-size, 0.01f, 0.f), glm::vec3(size, 0.01f, 0.f), white);
	drawer.add_debugline(glm::vec3(0.f, 0.01f, -size), glm::vec3(0.f, 0.01f, size), white);
	for (float x = 1.0f; x <= size; x++)
	{
		drawer.add_debugline(glm::vec3(-x, 0.01f, -size), glm::vec3(-x, 0.01f, size), red);
		drawer.add_debugline(glm::vec3( x, 0.01f, -size), glm::vec3( x, 0.01f, size), red);
		drawer.add_debugline(glm::vec3(-size, 0.01f, -x), glm::vec3(size, 0.01f, -x), blue);
		drawer.add_debugline(glm::vec3(-size, 0.01f,  x), glm::vec3(size, 0.01f,  x), blue);
	}

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

		glm::vec3 color = ((uint)m_hovered == i) ? magenta : black;
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
		drawer.add_debugline(info.m_pi, info.m_pi + 0.1f*info.m_normal, magenta);
		drawer.add_debugline_cube(mouse_ray.m_start, 0.0001f, white);
		m_hovered = static_cast<int>(info.m_body);

		if (input.m_mouse_triggered[0])
		{
			if (m_selected == m_hovered)
			{
				glm::vec3 force = glm::normalize(mouse_ray.m_direction);
				if (input.is_key_down(GLFW_KEY_LEFT_SHIFT))
					force *= 0.01f;
				b.add_impulse(force, info.m_pi);
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
	if (!ImGuizmo::IsOver())
	{
		if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
			object_picking();

		if (input.is_key_triggered(GLFW_KEY_R))
			reset_scene();
	}
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
			static ImGuizmo::OPERATION m_operation{ ImGuizmo::TRANSLATE };
			if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
			{
				if (input.is_key_triggered(GLFW_KEY_1))
					m_operation = ImGuizmo::TRANSLATE;
				if (input.is_key_triggered(GLFW_KEY_2))
					m_operation = ImGuizmo::ROTATE;
			}

			glm::mat4 model = b.get_model();
			ImGuizmo::SetRect(0, 0, (float)window.m_width, (float)window.m_height);
			ImGuizmo::BeginFrame();
			ImGuizmo::Manipulate(
				&drawer.m_camera.m_view[0][0],
				&drawer.m_camera.m_proj[0][0],
				m_operation, ImGuizmo::WORLD,
				&model[0][0], NULL, NULL);

			float matrixTranslation[3], matrixRotation[3], matrixScale[3];
			ImGuizmo::DecomposeMatrixToComponents(&model[0][0], matrixTranslation, matrixRotation, matrixScale);
			glm::vec3 eu_angles{ matrixRotation[0], matrixRotation[1], matrixRotation[2] };
			switch (m_operation)
			{
			case ImGuizmo::TRANSLATE:
				glm::vec3 goal{ matrixTranslation[0], matrixTranslation[1], matrixTranslation[2] };
				b.m_linear_momentum += (goal - b.m_position)*0.2f;
				break;
			case ImGuizmo::ROTATE:
				b.set_rotation(glm::normalize(glm::quat{ glm::radians(eu_angles) }));
				break;
			}

			ImGui::DragFloat3("Position", &b.m_position.x, 0.01f);
			if (ImGui::InputFloat4("Rotation", &b.m_rotation.x))
				b.m_rotation = glm::normalize(b.m_rotation);

			if (ImGui::Button("StopMovement"))
				b.clear_momentum();
			ImGui::SameLine();
			if (ImGui::RadioButton("Is Static", b.m_is_static))
				b.set_static(!b.m_is_static);

			ImGui::NewLine();
			ImGui::InputFloat3("Linear M", &b.m_position.x);
			ImGui::InputFloat3("Angular M", &b.m_position.x);
			ImGui::InputFloat("Mass", &b.m_inv_mass);
			
			ImGui::NewLine();

			glm::mat3 i = glm::inverse(b.m_inv_inertia);
			ImGui::InputFloat3("Inertia", &i[0].x);
			ImGui::InputFloat3("", &i[1].x);
			ImGui::InputFloat3("", &i[2].x);
			ImGui::End();
		}
	}

	if (ImGui::Begin("Physics", nullptr))
	{
		ImGui::Checkbox("Minkowski", &physics.m_draw_minkowski);
		ImGui::Checkbox("GJK Simplex", &physics.m_draw_gjk_simplex);
		int gjk_it = gjk::c_max_iterations;
		if (ImGui::SliderInt("GJK It", &gjk_it, 0, 64))
			gjk::c_max_iterations = gjk_it;

		ImGui::Checkbox("EPA Simplex", &physics.m_draw_epa_simplex);
		ImGui::Checkbox("EPA Polytope", &physics.m_draw_epa_polytope);
		ImGui::Checkbox("EPA Results", &physics.m_draw_epa_results);
		int epa_it = epa::c_max_iterations;
		if(ImGui::SliderInt("EPA It", &epa_it, 0, 256))
			epa::c_max_iterations = epa_it;
		ImGui::End();
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

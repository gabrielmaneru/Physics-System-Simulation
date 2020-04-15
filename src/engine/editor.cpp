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
	ImGui::GetStyle().Colors[2] = ImVec4{ 0.75f, 1.0f, 0.5f, 0.6f };
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
		.set_static(true)
		.set_friction(m_floor_friction)
		.set_restitution(m_floor_restitution);
	physics.m_meshes.back().scale(40.f);
	physics.add_body("cube.obj")
		.set_position({ 40.0f, 0.0f, 0.0f })
		.set_static(true)
		.set_friction(m_floor_friction)
		.set_restitution(m_floor_restitution);
	physics.m_meshes.back().scale(40.f);
	physics.add_body("cube.obj")
		.set_position({ -40.0f, 0.0f, 0.0f })
		.set_static(true)
		.set_friction(m_floor_friction)
		.set_restitution(m_floor_restitution);
	physics.m_meshes.back().scale(40.f);
	physics.add_body("cube.obj")
		.set_position({ 0.0f, 0.0f, 40.0f })
		.set_static(true)
		.set_friction(m_floor_friction)
		.set_restitution(m_floor_restitution);
	physics.m_meshes.back().scale(40.f);
	physics.add_body("cube.obj")
		.set_position({ 0.0f, 0.0f, -40.0f })
		.set_static(true)
		.set_friction(m_floor_friction)
		.set_restitution(m_floor_restitution);
	physics.m_meshes.back().scale(40.f);

	switch (m_scene)
	{
	case 0:
		physics.add_body("cube.obj")
			.set_position({ 0.0f, 2.0f, 0.0f })
			.set_friction(m_general_friction)
			.set_restitution(m_general_restitution);
		
		physics.add_body("cube.obj")
			.set_position({ 0.0f, 1.0f, 0.0f })
			.set_rotation(glm::quat{ glm::vec3{0.0f, glm::half_pi<float>(), 0.0f} })
			.set_static(true)
			.set_friction(m_general_friction)
			.set_restitution(m_general_restitution);
	break;


	case 1:
	physics.add_body("cube.obj")
		.set_position({ 10.f, 0.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f)
		.add_impulse_linear(glm::vec3(-m_general_impulse, 0.0f, 0.0f));
	physics.add_body("cube.obj")
		.set_position({ 0.f, 0.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ -1.f, 0.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 10.f, 1.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f)
		.add_impulse_linear(glm::vec3(-m_general_impulse, 0.0f, 0.0f));
	physics.add_body("cube.obj")
		.set_position({ 0.f, 1.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ -1.f, 1.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ 10.f, 2.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f)
		.add_impulse_linear(glm::vec3(-m_general_impulse, 0.0f, 0.0f));
	physics.add_body("cube.obj")
		.set_position({ 0.f, 2.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f);
	physics.add_body("cube.obj")
		.set_position({ -1.f, 2.5f, 0.0f })
		.set_friction(m_general_friction)
		.set_restitution(m_general_restitution)
		.set_mass(1.0f);
		break;


	case 2:
		for (float i = 0.0f; i <= 1.0f; i += 0.1f)
		{
			physics.add_body("cube.obj")
				.set_position({ -10.0f + 20.0f*i, 1.f, 0.0f })
				.set_friction(m_general_friction*i)
				.set_restitution(m_general_restitution*i)
				.set_mass(1.0f)
				.set_static(true);
			physics.add_body("cube.obj")
				.set_position({ -10.0f + 20.0f*i, 2.f, 0.0f })
				.set_friction(m_general_friction*i)
				.set_restitution(m_general_restitution*i)
				.set_mass(1.0f)
				.add_impulse_linear(glm::vec3(0.0f, 0.0f, m_general_impulse));
		}
		break;

	case 3:
		for (float i = 0.0f; i <= 1.0f; i += 0.1f)
		{
			physics.add_body("cube.obj")
				.set_position({ -10.0f + 20.0f*i, 1.f, 0.0f })
				.set_friction(m_general_friction)
				.set_mass(1.0f)
				.set_static(true);
			physics.add_body("sphere.obj")
				.set_position({ -10.0f + 20.0f*i, 2.f, 0.0f })
				.set_friction(m_general_friction)
				.set_roll(m_general_roll*i)
				.set_mass(1.0f)
				.add_impulse_linear(glm::vec3(0.0f, 0.0f, m_general_impulse));
		}
		break;

	case 4:
		for (float i = 0.0f; i <= 20.0f; ++i)
		{
			physics.add_body("cube.obj")
				.set_position({ 0.0f, + 0.5f+i, 0.0f })
				.set_friction(m_general_friction*i)
				.set_restitution(m_general_restitution*i)
				.set_mass(1.0f);
		}
		break;

	case 5:
		for (uint i = 0; i < 50u; ++i)
			physics.add_body((i % 2 == 0) ? "cube.obj" : "sphere.obj")
			.set_position({ rand(-2.5f, 2.5f), rand(1.f, 20.f), rand(-2.5f, 2.5f) })
			.set_rotation(glm::normalize(glm::quat{ glm::vec3{
				rand(0.0f, glm::half_pi<float>()),
				rand(0.0f, glm::half_pi<float>()),
				rand(0.0f, glm::half_pi<float>())
			} }))
			.set_friction(m_general_friction)
			.set_restitution(m_general_restitution)
			.set_roll((i % 2 == 0) ? 0.0f : m_general_roll)
			.set_mass(1.0f);
	default:
		break;
	}
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
	if (m_wireframe)
	{
		float size = 20.0f;
		drawer.add_debugline(glm::vec3(-size, 0.01f, 0.f), glm::vec3(size, 0.01f, 0.f), white);
		drawer.add_debugline(glm::vec3(0.f, 0.01f, -size), glm::vec3(0.f, 0.01f, size), white);
		for (float x = 1.0f; x <= size; x++)
		{
			drawer.add_debugline(glm::vec3(-x, 0.01f, -size), glm::vec3(-x, 0.01f, size), red);
			drawer.add_debugline(glm::vec3(x, 0.01f, -size), glm::vec3(x, 0.01f, size), red);
			drawer.add_debugline(glm::vec3(-size, 0.01f, -x), glm::vec3(size, 0.01f, -x), blue);
			drawer.add_debugline(glm::vec3(-size, 0.01f, x), glm::vec3(size, 0.01f, x), blue);
		}
	}

	for (uint i = 0; i < physics.m_bodies.size(); i++)
	{
		const physical_mesh& mesh = physics.m_meshes[i];
		std::vector<glm::vec3> lines = mesh.get_lines();
		std::pair<std::vector<glm::vec3>,
			std::vector<glm::vec3> > tri = mesh.get_triangles();

		const body& bdy = physics.m_bodies[i];
		glm::mat4 m = bdy.get_model();
		for (auto& p : lines)
			p = tr_point(m, p);
		for (uint i = 0; i < tri.first.size(); ++i)
			tri.first[i] = tr_point(m, tri.first[i]),
			tri.second[i] = tr_vector(m, tri.second[i]);

		glm::vec3 color = ((uint)m_hovered == i) ? magenta : black;
		drawer.add_debugline_list(lines, color);
		drawer.add_debugtri_list(tri.first, tri.second, white);

		if (m_wireframe)
		{
			drawer.add_debugline(bdy.m_position, bdy.m_position + bdy.m_linear_momentum, green);
			drawer.add_debugline(bdy.m_position, bdy.m_position + bdy.m_angular_momentum, blue);
		}
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
				b.add_impulse_angular(glm::cross(info.m_pi - b.m_position, force));
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

		if (input.is_key_triggered(GLFW_KEY_F1))
			m_wireframe = !m_wireframe;
	}
}
void c_editor::drawGui()
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
			ImGui::InputFloat3("Linear M", &b.m_linear_momentum.x);
			ImGui::InputFloat3("Angular M", &b.m_angular_momentum.x);
			ImGui::InputFloat("Mass", &b.m_inv_mass);
			
			ImGui::NewLine();

			glm::mat3 i = glm::inverse(b.m_inv_inertia);
			ImGui::InputFloat3("Inertia", &i[0].x);
			ImGui::InputFloat3("", &i[1].x);
			ImGui::InputFloat3("", &i[2].x);
			ImGui::End();
		}
	}

	if (ImGui::Begin("Menu", nullptr))
	{
		if (ImGui::Button("Scene 0")) { m_scene = 0; reset_scene(); }
		ImGui::SameLine();
		if (ImGui::Button("Scene 1")) { m_scene = 1; reset_scene(); }
		ImGui::SameLine();
		if (ImGui::Button("Scene 2")) { m_scene = 2; reset_scene(); }
		ImGui::SameLine();
		if (ImGui::Button("Scene 3")) { m_scene = 3; reset_scene(); }
		ImGui::SameLine();
		if (ImGui::Button("Scene 4")) { m_scene = 4; reset_scene(); }
		ImGui::SameLine();
		if (ImGui::Button("Scene 5")) { m_scene = 5; reset_scene(); }
		ImGui::NewLine();
		ImGui::SliderFloat("Floor Friction", &m_floor_friction, 0.0f, 1.0f);
		ImGui::SliderFloat("Floor Restitution", &m_floor_restitution, 0.0f, 1.0f);
		ImGui::NewLine();

		switch (m_scene)
		{
		case 0:
			ImGui::SliderFloat("General Friction", &m_general_friction, 0.0f, 1.0f);
			ImGui::SliderFloat("General Restitution", &m_general_restitution, 0.0f, 1.0f);
			break;

		case 1:
			ImGui::SliderFloat("General Friction", &m_general_friction, 0.0f, 1.0f);
			ImGui::SliderFloat("General Restitution", &m_general_restitution, 0.0f, 1.0f);
			ImGui::SliderFloat("General Impulse", &m_general_impulse, 0.0f, 100.0f);
			break;


		case 2:
			ImGui::SliderFloat("General Friction", &m_general_friction, 0.0f, 1.0f);
			ImGui::SliderFloat("General Restitution", &m_general_restitution, 0.0f, 1.0f);
			ImGui::SliderFloat("General Impulse", &m_general_impulse, 0.0f, 100.0f);
			break;

		case 3:
			ImGui::SliderFloat("General Friction", &m_general_friction, 0.0f, 1.0f);
			ImGui::SliderFloat("General Restitution", &m_general_restitution, 0.0f, 1.0f);
			ImGui::SliderFloat("General Roll", &m_general_roll, 0.0f, 1.0f);
			ImGui::SliderFloat("General Impulse", &m_general_impulse, 0.0f, 100.0f);
			break;

		case 4:
			ImGui::SliderFloat("General Friction", &m_general_friction, 0.0f, 1.0f);
			ImGui::SliderFloat("General Restitution", &m_general_restitution, 0.0f, 1.0f);
			ImGui::NewLine();
			ImGui::SliderInt("Solver Iterations", &m_solver_iterations, 1, 100);
			ImGui::SliderFloat("Baumgarte Value", &m_baumgarte, 0.0f, 1.0f);
			ImGui::Checkbox("Do Warm Start", &m_do_warm_start);
			break;

		case 5:
			ImGui::SliderFloat("General Friction", &m_general_friction, 0.0f, 1.0f);
			ImGui::SliderFloat("General Restitution", &m_general_restitution, 0.0f, 1.0f);
			ImGui::SliderFloat("General Roll", &m_general_roll, 0.0f, 1.0f);
			ImGui::SliderFloat("General Impulse", &m_general_impulse, 0.0f, 100.0f);
			ImGui::NewLine();
			ImGui::SliderInt("Solver Iterations", &m_solver_iterations, 1, 100);
			ImGui::SliderFloat("Baumgarte Value", &m_baumgarte, 0.0f, 1.0f);
			ImGui::Checkbox("Do Warm Start", &m_do_warm_start);
		default:
			break;
		}

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

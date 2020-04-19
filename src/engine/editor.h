/**
 * @file editor.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief GUI Manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once

class c_editor
{
	int m_selected{ -1 };
	int m_hovered{ -1 };
	int m_scene{ 0 };
	float m_floor_friction{ 0.0f };
	float m_floor_restitution{ 0.2f };
	float m_general_friction{ 0.0f };
	float m_general_restitution{ 0.2f };
	float m_general_roll{ 0.0f };
	float m_general_impulse{ 30.0f };

	bool imgui_initialize()const;
	void imgui_shutdown()const;
	void create_scene()const;
	void reset_scene();
	void draw_debug_bodies()const;
	void object_picking();

public:
	int m_solver_iterations{ 20 };
	float m_baumgarte{ 0.05f };
	bool m_do_warm_start{ true };
	bool m_wireframe{ false };

	bool initialize();
	void update();
	void drawGui();
	void shutdown();
	static c_editor& get_instance();
};
#define editor c_editor::get_instance()
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
	bool ImGui_Init()const;
	void ImGui_Shutdown()const;
	void create_scene()const;
	void reset_scene();
	void draw_debug_bodies()const;
	void object_picking();
	int m_selected{ -1 };
	int m_hovered{ -1 };

public:
	bool initialize();
	void update();
	void drawGui()const;
	void shutdown();
	static c_editor& get_instance();
};
#define editor c_editor::get_instance()
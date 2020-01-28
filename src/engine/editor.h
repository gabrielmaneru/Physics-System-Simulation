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
public:
	bool initialize();
	void drawGUI();
	void shutdown();
	static c_editor& get_instance();
};
#define editor c_editor::get_instance()
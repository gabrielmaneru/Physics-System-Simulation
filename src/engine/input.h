/**
 * @file input.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Input manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <unordered_set>

struct GLFWwindow;
struct s_input
{
	void setup_callback(GLFWwindow*)const;
	void clear_triggers();
	bool is_key_up(int key)const;
	bool is_key_down(int key)const;
	bool is_key_triggered(int key)const;
	bool is_key_released(int key)const;
	static s_input& get_instance();

	std::unordered_set<int> m_triggered_keys;
	int m_keyboard[348];
	double m_mouse_pos[2];
	double m_mouse_offset[2];
	bool m_mouse_pressed[2];
	bool m_mouse_triggered[2];
};
#define input s_input::get_instance()

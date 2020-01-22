#pragma once
#include "camera.h"

class Shader_Program;
class c_drawer
{
	camera m_camera;
	Shader_Program* color_shader;

public:
	bool initialize();
	void render();
	void shutdown();
	static c_drawer& get_instance();
};
#define drawer c_drawer::get_instance()
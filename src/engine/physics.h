#pragma once
#include "body.h"
#include "raw_mesh.h"
#include <physics/physical_mesh.h>
#include <physics/ray.h>
#include <map>

class c_physics
{
	float ray_cast(const ray&)const;
	void add_debug_lines()const;

	std::vector<physical_mesh> m_meshes;
	std::vector<body> m_bodies;
	std::map<std::string, raw_mesh> m_loaded_meshes;

public:
	bool initialize();
	void update();
	void shutdown();
	void add_body(const body& b, std::string file);
	static c_physics& get_instance();
};
#define physics c_physics::get_instance()
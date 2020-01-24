#pragma once
#include "body.h"
#include "raw_mesh.h"
#include <physics/physical_mesh.h>
#include <physics/ray.h>
#include <map>

struct ray_info_detailed : public ray_info
{
	glm::vec3 m_pi;
	uint m_body;
};

class c_physics
{
	ray_info_detailed ray_cast(const ray&)const;
	void draw_debug_bodies()const;

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

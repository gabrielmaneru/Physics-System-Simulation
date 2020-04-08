/**
 * @file physics.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Physics system manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include "raw_mesh.h"
#include <physics/physical_mesh.h>
#include <physics/body.h>
#include <physics/contact_info.h>
#include <physics/ray.h>
#include <map>
#include <array>

struct ray_info_detailed : public ray_info
{
	glm::vec3 m_pi;
	uint m_body;
};

class c_physics
{
	ray_info_detailed ray_cast(const ray&)const;
	contact collision_narrow(const physical_mesh& m1,
		const physical_mesh& m2,
		body& b1,
		body& b2)const;
	std::vector<physical_mesh> m_meshes;
	std::vector<body> m_bodies;
	std::map<std::string, raw_mesh> m_loaded_meshes;
	glm::vec3 m_gravity{ 0.f, -1.f, 0.f };

public:
	void update();
	void clean();
	body& add_body(std::string file);

	bool m_draw_minkowski{false};
	bool m_draw_gjk_simplex{ false };
	bool m_draw_epa_simplex{ false };
	bool m_draw_epa_polytope{ false };
	bool m_draw_epa_results{ false };

	static c_physics& get_instance();
	friend class c_editor;
};
#define physics c_physics::get_instance()

/**
 * @file physical_mesh.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Physical mesh structure using half-edge representation
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include "face.h"
#include "half_edge.h"
#include "ray.h"
#include <vector>
#include <list>
#include <glm/glm.hpp>

struct ray_info
{
	bool m_intersected{ false };
	float m_time{FLT_MAX};
	glm::vec3 m_normal;
};

struct physical_mesh
{
	physical_mesh() = default;
	physical_mesh(physical_mesh&&);

	std::vector<glm::vec3> m_vertices;
	std::list<half_edge> m_hedges;
	std::list<face> m_faces;

	void add_face(const std::vector<uint>& indices);
	void create_twins();
	void merge_coplanar();
	void remove_edge(half_edge*);

	std::vector<glm::vec3> get_lines()const;
	std::vector<glm::vec3> get_triangles()const;
	glm::vec4 get_face_plane(const half_edge* hedge)const;
	bool is_coplanar(const half_edge* hedge)const;
	ray_info ray_cast(const ray& local_ray)const;
	glm::vec3 support_point_bruteforce(glm::vec3 dir)const;
	glm::vec3 support_point_hillclimb(glm::vec3 dir)const;
};
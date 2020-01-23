#pragma once
#include "vertex.h"
#include "face.h"
#include "half_edge.h"
#include <vector>
#include <list>

struct physical_mesh
{
	physical_mesh() = default;
	physical_mesh(physical_mesh&&);
	std::vector<vertex> m_vertices;
	std::list<half_edge> m_hedges;
	std::list<face> m_faces;

	void add_face(const std::vector<uint>& indices);
	void create_twins();
	void merge_coplanar();
	void remove_edge(half_edge*);

	glm::vec4 get_face_plane(const half_edge* hedge)const;
	bool is_coplanar(const half_edge* hedge)const;
};
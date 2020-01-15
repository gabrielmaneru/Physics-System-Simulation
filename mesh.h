#pragma once
#include "vertex.h"
#include "face.h"
#include "half_edge.h"
#include <vector>

struct half_edge_mesh
{
	std::vector<vertex> m_vertices;
	std::vector<face> m_faces;
	std::vector<half_edge> m_hedges;

	void add_face(const std::vector<unsigned int>&)
};
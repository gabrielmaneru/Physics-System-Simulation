#include <gtest/gtest.h>
#include "mesh.h"

TEST(half_edge, coplanarity_quad)
{
	half_edge_mesh mesh{};
	mesh.m_vertices.push_back({ {0.0f,0.0f,0.0f} });
	mesh.m_vertices.push_back({ {1.0f,0.0f,0.0f} });
	mesh.m_vertices.push_back({ {1.0f,1.0f,0.0f} });
	mesh.m_vertices.push_back({ {0.0f,1.0f,0.0f} });
	mesh.m_faces.push_back({ {0u,1u,2u} });
	mesh.m_faces.push_back({ {2u,3u,0u} });


	ASSERT_TRUE(false);
}


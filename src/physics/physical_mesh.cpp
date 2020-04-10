/**
 * @file physical_mesh.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Physical mesh structure using half-edge representation
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "physical_mesh.h"
#include "math_utils.h"
#include <engine/drawer.h>

/**
 * Move constructor for the mesh(in order to later build a vector of meshes)
**/
physical_mesh::physical_mesh(physical_mesh && o)
	:m_vertices(std::move(o.m_vertices)),
	m_hedges(std::move(o.m_hedges)),
	m_faces(std::move(o.m_faces))
{
	for (auto&f : m_faces)
		f.m_owner = this;
}

/**
 * Add a new face to the mesh
**/
void physical_mesh::add_face(const std::vector<uint>& indices)
{
	// Assert minimum face is a triangle
	assert(indices.size() >= 3);

	// Create all half edges
	half_edge* start{ nullptr };
	half_edge* last{ nullptr };
	for (uint i = 0; i < indices.size(); i++)
	{
		// Create a half_edge
		m_hedges.push_back({});
		half_edge * cur = &m_hedges.back();

		// Initialize
		cur->m_vertex_idx = indices[i];

		// Set up double linked-list
		if (last != nullptr)
		{
			cur->m_prev = last;
			last->m_next = cur;
		}
		else
			start = cur;

		cur->m_twin = nullptr;
		last = cur;
	}

	// Finish cyclic linked-list
	start->m_prev = last;
	last->m_next = start;

	// Create a face
	m_faces.push_back({this});
	m_faces.back().m_hedge_start = start;
	m_faces.back().refresh();
}
/**
 * Indentify the twin edges in the mesh
**/
void physical_mesh::create_twins()
{
	// For each face
	std::list<face>::const_iterator it = m_faces.cbegin();
	for (; it != m_faces.end(); it++)
	{
		// Get Start of linked list
		half_edge * edge1 = it->m_hedge_start;

		// Do one cycle through the list
		do
		{
			// For each other face until you find a twin
			std::list<face>::const_iterator other = it;
			for (other++; other != m_faces.end() && edge1->m_twin == nullptr; other++)
			{
				// Get Start of other linked list
				half_edge * edge2 = other->m_hedge_start;

				// Do one cycle through the other list
				do
				{
					if (edge2->m_twin == nullptr
					&&  edge2->get_start() == edge1->get_end()
					&&  edge2->get_end() == edge1->get_start())
					{
						edge1->m_twin = edge2;
						edge2->m_twin = edge1;
						break;
					}
					edge2 = edge2->m_next;
				} while (edge2 != other->m_hedge_start);
			}
			edge1 = edge1->m_next;
		} while (edge1 != it->m_hedge_start);
	}
}
/**
 * Merges coplanar faces
**/
void physical_mesh::merge_coplanar()
{
	// For each face
	for (auto it = m_faces.begin(); it != m_faces.end(); it++)
	{
		// Check face exists
		if (it->m_hedge_start != nullptr)
		{
			// Get Start of linked list
			half_edge * edge1 = it->m_hedge_start;

			// Do one cycle through the list
			do
			{
				// If the edge has a twin
				if (edge1->m_twin != nullptr && is_coplanar(edge1))
				{
					half_edge * new_start{ nullptr };

					// If twin is prev to edge1
					if (edge1->m_prev == edge1->m_twin)
					{
						edge1->m_next->m_prev = edge1->m_twin->m_prev;
						edge1->m_twin->m_prev->m_next = edge1->m_next;

						new_start = edge1->m_next;
					}

					// If twin is next to edge1
					else if (edge1->m_next == edge1->m_twin)
					{
						edge1->m_prev->m_next = edge1->m_twin->m_next;
						edge1->m_twin->m_next->m_prev = edge1->m_prev;

						new_start = edge1->m_prev;
					}

					// Merge faces
					else
					{
						// Change Current bounds
						edge1->m_prev->m_next = edge1->m_twin->m_next;
						edge1->m_next->m_prev = edge1->m_twin->m_prev;

						// Change twin bounds
						edge1->m_twin->m_next->m_prev = edge1->m_prev;
						edge1->m_twin->m_prev->m_next = edge1->m_next;

						// Reconnect the new face
						new_start = edge1->m_prev;

						// Remove other face
						edge1->m_twin->m_face->m_hedge_start = nullptr;
					}

					// Create a face
					m_faces.push_back({this});
					m_faces.back().m_hedge_start = new_start;
					m_faces.back().refresh();

					// Remove current face
					it->m_hedge_start = nullptr;

					// Remove both hedges
					remove_edge(edge1->m_twin);
					remove_edge(edge1);
					break;
				}
				else
					edge1 = edge1->m_next;
			} while (edge1 != it->m_hedge_start);
		}
	}
	// Delete null faces
	for (auto it = m_faces.begin(); it != m_faces.end();)
	{
		if (it->m_hedge_start == nullptr)
			it = m_faces.erase(it);
		else
			it++;
	}

}
/**
 * Helper function for remove edges
**/
void physical_mesh::remove_edge(half_edge * hedge)
{
	std::list<half_edge>::iterator it = m_hedges.begin();
	for (; it != m_hedges.end(); it++)
		if (&*it == hedge)
		{
			m_hedges.erase(it);
			return;
		}
}
/**
 * Scale the vertices
**/
void physical_mesh::scale(float s)
{
	for (auto& v : m_vertices)
		v *= s;
	for (auto& f : m_faces)
		f.refresh();
}

/**
 * Extract the lines of the mesh
**/
std::vector<glm::vec3> physical_mesh::get_lines() const
{
	std::vector<glm::vec3> lines;
	for (auto f : m_faces)
	{
		half_edge* hedge = f.m_hedge_start;
		do
		{
			lines.push_back(m_vertices[hedge->get_start()]);
			lines.push_back(m_vertices[hedge->get_end()]);
			hedge = hedge->m_next;
		} while (hedge != f.m_hedge_start);
	}
	return lines;
}
/**
 * Extract the triangles of the mesh
**/
std::vector<glm::vec3> physical_mesh::get_triangles() const
{
	std::vector<glm::vec3> tri;
	for (auto f : m_faces)
	{
		half_edge* hedge = f.m_hedge_start;
		glm::vec3 fan_0 = m_vertices[hedge->get_start()];
		hedge = hedge->m_next;

		do
		{
			tri.push_back(fan_0);
			tri.push_back(m_vertices[hedge->get_start()]);
			tri.push_back(m_vertices[hedge->get_end()]);
			hedge = hedge->m_next;
		} while (hedge != f.m_hedge_start);
	}
	return tri;
}
/**
 * Extract the plane of a face
**/
 glm::vec4 physical_mesh::get_face_plane(const half_edge * hedge) const
{
	glm::vec3 p0 = m_vertices[hedge->get_start()];
	glm::vec3 p1 = m_vertices[hedge->get_end()];
	glm::vec3 p2 = m_vertices[hedge->get_other()];

	glm::vec3 v0 = glm::normalize(p1 - p0);
	glm::vec3 v1 = glm::normalize(p2 - p1);

	glm::vec3 n = glm::cross(v0, v1);
	if (glm::length2(n) == 0.0f)
		return glm::vec4{0.0f};

	n = glm::normalize(n);
	return glm::vec4(n.x, n.y, n.z, glm::dot(n, p0));
}
/**
 * Check if two twin hedges are coplanar
**/
bool physical_mesh::is_coplanar(const half_edge * hedge) const
{
	if (hedge->m_twin == nullptr)
		return false;

	glm::vec4 plane1 = get_face_plane(hedge);
	glm::vec3 p_in_edge2 = m_vertices[hedge->m_twin->get_other()];
	float dot = glm::dot(glm::vec3(plane1), p_in_edge2);
	float dist_to_plane = dot - plane1.w;
	return abs(dist_to_plane) < c_epsilon;
}
/**
 * Perform ray intersection against the mesh
**/
ray_info physical_mesh::ray_cast(const ray & local_ray)const
{
	ray_info info;
	for (auto f : m_faces)
	{
		glm::vec4 face_plane = get_face_plane(f.m_hedge_start);

		// Intersect the plane of the face
		float time = local_ray.ray_cast_plane(face_plane);
		if (time >= 0.0f && time < info.m_time)
		{
			glm::vec3 proj_point{ local_ray.get_point(time) };
			glm::vec3 p0 = m_vertices[f.m_indices[0]];

			// Triangulate the face (as a fan)
			for (uint tri = 1; tri < f.m_indices.size() - 1; tri++)
			{
				glm::vec3 p1 = m_vertices[f.m_indices[tri]];
				glm::vec3 p2 = m_vertices[f.m_indices[tri+1]];

				glm::vec3 v1 = p1 - p0;
				glm::vec3 v2 = p2 - p0;

				float d_v1 = glm::dot(v1, v1);
				float d_v2 = glm::dot(v2, v2);
				float d_v1_v2 = glm::dot(v1, v2);

				// Check the affine coordinates to see if the ray
				// lies inside the triangle
				float d = d_v1*d_v2 - d_v1_v2*d_v1_v2;
				if (glm::abs(d) > c_epsilon)
				{
					glm::vec3 p = proj_point - p0;
					float d_p_v1 = glm::dot(v1, p);
					float d_p_v2 = glm::dot(v2, p);

					float s = (d_v2*d_p_v1 - d_v1_v2 * d_p_v2) / d;
					float t = (d_v1*d_p_v2 - d_v1_v2 * d_p_v1) / d;

					if (s+t < 1.0f && 0.0f <= s && 0.0f <= t)
					{
						info.m_intersected = true;
						info.m_time = time;
						info.m_normal = glm::vec3(get_face_plane(f.m_hedge_start));
						break;
					}
				}
			}
		}
	}
	return info;
}
 glm::vec3 physical_mesh::support(glm::vec3 dir) const
 {
	return support_point_hillclimb(dir);
 }
/**
* Computes the support point using a bruteforce approach
**/
glm::vec3 physical_mesh::support_point_bruteforce(glm::vec3 dir)const
{
	glm::vec3 sp = m_vertices[0];
	float dist = glm::dot(sp, dir);

	for (uint i = 1; i < m_vertices.size(); ++i)
	{
		float d = glm::dot(m_vertices[i], dir);
		if (d > dist)
			dist = d, sp = m_vertices[i];
	}
	return sp;
}
glm::vec3 physical_mesh::support_point_hillclimb(glm::vec3 dir, const half_edge * start) const
{
	if (start == nullptr)
		start = &m_hedges.front();

	const half_edge * best = start;
	float dist = glm::dot(m_vertices[start->get_end()], dir);

	const half_edge * it{ start};
	do
	{
		if (it->m_twin == nullptr)
			break;

		float d_it = glm::dot(m_vertices[it->m_twin->get_end()], dir);
		if (d_it > dist)
			dist = d_it, best = it->m_twin;
		it = it->m_twin->m_prev;
	} while (it != start);
	if (start != best)
		return support_point_hillclimb(dir, best);
	else
		return m_vertices[best->get_end()];
}

const face * physical_mesh::find_most_antiparallel_face(const glm::vec3 & dir) const
{
	float min_dot{ FLT_MAX };
	const face* most{ nullptr };

	for (auto it = m_faces.cbegin(); it != m_faces.cend(); ++it)
	{
		const glm::vec3 n = it->m_plane;
		float dot = glm::dot(n, dir);
		if (dot < min_dot)
			min_dot = dot,
			most = &*it;
	}
	return most;
}

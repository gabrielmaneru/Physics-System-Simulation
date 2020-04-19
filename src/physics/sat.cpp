/**
 * @file sat.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Separating Axis Theorem algorithm
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "sat.h"
#include "contact_info.h"
#include "physical_mesh.h"
#include "body.h"
#include "math_utils.h"
#include <glm/glm.hpp>

sat::sat(const overlap_pair * pair)
:   bA(pair->body_A), bB(pair->body_B),
	mA(pair->mesh_A), mB(pair->mesh_B),
	trAtoWorld(bA->get_model()), trBtoWorld(bB->get_model()),
	trAtoB(bB->get_invmodel() * bA->get_model()),
	trBtoA(glm::inverse(trAtoB)),
	was_colliding{pair->m_state == overlap_pair::state::Collision},
	prev_data{ pair->prev_data }, next_data{ pair->prev_data }
{}

sat::result sat::test_collision()
{
	// Check previous frame separating axis
	if (prev_data.m_actor != actor::Null)
	{
		float penetration{0.0f};
		bool valid_penetration{ true };
		switch (prev_data.m_actor)
		{
		case sat::actor::A:
			penetration = compute_face_penetration(mB, trAtoB, prev_data.m_face);
			break;
		case sat::actor::B:
			penetration = compute_face_penetration(mA, trBtoA, prev_data.m_face);
			break;
		case sat::actor::Edge:
			{
#if 0 // TODO: Fix edge cached data for fast-checking
				const half_edge* edge1{ prev_data.m_edgeA };
				const glm::vec3 edge1_start = tr_point(trAtoB, mA->m_vertices[edge1->get_start()]);
				const glm::vec3 edge1_end = tr_point(trAtoB, mA->m_vertices[edge1->get_end()]);
				const glm::vec3 edge1_dir = edge1_end - edge1_start;
				const glm::vec3 edge1_normal = tr_vector(trAtoB, edge1->m_face->m_plane);
				const glm::vec3 edge1_twinnormal = tr_vector(trAtoB, edge1->m_twin->m_face->m_plane);
				
				const half_edge* edge2{ prev_data.m_edgeB };
				const glm::vec3 edge2_start = mB->m_vertices[edge2->get_start()];
				const glm::vec3 edge2_end = mB->m_vertices[edge2->get_end()];
				const glm::vec3 edge2_dir = edge2_end - edge2_start;
				
				const glm::vec3 edge2_normal = edge2->m_face->m_plane;
				const glm::vec3 edge2_twinnormal = edge2->m_twin->m_face->m_plane;
				
				if (test_gaussmap_intersect(edge1_normal, edge1_twinnormal, -edge2_normal, -edge2_twinnormal, -edge1_dir, -edge2_dir))
				{
					const glm::vec3 centroidA = tr_point(trAtoB, glm::vec3{ 0.0f });
					penetration = compute_edge_penetration(edge1_start, edge2_start, centroidA, edge1_dir, edge2_dir);
				}
				else
#endif
					valid_penetration = false;
			}
			break;
		}

		// Check if the penetration is valid
		if (valid_penetration)
		{
			// Check if there are still separating
			if (!was_colliding && penetration <= 0.0f)
				return{};
			if (was_colliding && penetration > 0.0f)
			{
				next_data.m_penetration = penetration;
				return generate_manifold(next_data);
			}
		}
	}
	// Compute face normals of A as separating axis
	penetration_data face_A_pen = test_faces(actor::A);
	// Check if negative penetration -> separating axis
	if (face_A_pen.m_penetration <= c_epsilon)
	{
		next_data = face_A_pen;
		return {};
	}
	// Check face normals of B as separating axis
	penetration_data face_B_pen = test_faces(actor::B);
	// Check if negative penetration -> separating axis
	if (face_B_pen.m_penetration <= c_epsilon)
	{
		next_data = face_B_pen;
		return {};
	}
	// Check edge vs edge
	penetration_data edge_pen = test_edges();
	// Check if negative penetration -> separating axis
	if (edge_pen.m_penetration <= c_epsilon)
	{
		next_data = edge_pen;
		return {};
	}
	// Minimum penetration info
	penetration_data min_penetration;
	// Check minimum face penetration axis
	// Bias the result for better consistency
	if (face_A_pen.m_penetration < face_B_pen.m_penetration * 1.005 + 0.005)
		min_penetration = face_A_pen;
	else
		min_penetration = face_B_pen;
	// Check if minimum penetration axis is edge
	if (edge_pen.m_penetration * 1.005 + 0.005 < min_penetration.m_penetration)
		min_penetration = edge_pen;
	// If minimum_penetration not found (alignment corner case)
	if (min_penetration.m_penetration == FLT_MAX)
		return {};
	//  Cach minimum penetration
	next_data = min_penetration;
	// generate & return manifold
	return generate_manifold(min_penetration);
}
sat::penetration_data sat::test_faces(actor a)
{
	// Get current data
	bool actor_is_A{ a == actor::A };
	const physical_mesh* mCur{ actor_is_A ? mA : mB };
	const physical_mesh* mOther{ actor_is_A ? mB : mA };
	const glm::mat4& trCurToOther{ actor_is_A ? trAtoB : trBtoA };
	// Minimum penetration info
	penetration_data min_penetration{a};
	// For each face in acting mesh
	for (auto it = mCur->m_faces.cbegin(); it != mCur->m_faces.cend(); ++it)
	{
		// Get current face
		const face* f = &*it;
		// Compute face penetration
		const float penetration = compute_face_penetration(mOther, trCurToOther, f);
		// If penetration is negative -> Found a separating axis
		if (penetration < 0.0f)
		{
			penetration_data face_pen{ a,penetration };
			face_pen.m_face = f;
			return face_pen;
		}
		// Store minimum penetration
		if (penetration < min_penetration.m_penetration)
		{
			min_penetration.m_face = f;
			min_penetration.m_penetration = penetration;
		}
	}
	// Return minimum penetration
	return min_penetration;
}
float sat::compute_face_penetration(const physical_mesh * other, const glm::mat4 tr, const face * face)
{
	// Get face data
	const glm::vec4& planeA = face->m_plane;
	const glm::vec3 normalA = glm::vec3(planeA);
	const glm::vec3 pointA = normalA * planeA.w;
	// Transform to B space
	const glm::vec3 normalB = tr_vector(tr, normalA);
	const glm::vec3 pointB = tr_point(tr, pointA);
	// Get support point
	const glm::vec3 supp = other->support(-normalB);
	// Compute penetration
	return glm::dot(pointB - supp, normalB);
}
sat::penetration_data sat::test_edges()
{
	// Minimum penetration info
	penetration_data min_penetration{ actor::Edge };
	// For each face in mesh A
	for (auto itA = mA->m_hedges.cbegin(); itA != mA->m_hedges.cend(); ++itA)
	{
		// Get current edge 1
		const half_edge* edge1{ &*itA };
		// Avoid second computation for twin
		if (edge1->m_twin < edge1)
			continue;
		// Get edge 1 data
		const glm::vec3 edge1_start = tr_point(trAtoB, mA->m_vertices[edge1->get_start()]);
		const glm::vec3 edge1_end = tr_point(trAtoB, mA->m_vertices[edge1->get_end()]);
		const glm::vec3 edge1_dir = edge1_end - edge1_start;
		const glm::vec3 edge1_normal = tr_vector(trAtoB, edge1->m_face->m_plane);
		const glm::vec3 edge1_twinnormal = tr_vector(trAtoB, edge1->m_twin->m_face->m_plane);
		// For each face in mesh B
		for (auto itB = mB->m_hedges.cbegin(); itB != mB->m_hedges.cend(); ++itB)
		{
			// Get current edge 2
			const half_edge* edge2{ &*itB };
			// Avoid second computation for twin
			if (edge2->m_twin < edge2)
				continue;
			// Get edge 2 data
			const glm::vec3 edge2_start = mB->m_vertices[edge2->get_start()];
			const glm::vec3 edge2_end = mB->m_vertices[edge2->get_end()];
			const glm::vec3 edge2_dir = edge2_end - edge2_start;
			const glm::vec3 edge2_normal = edge2->m_face->m_plane;
			const glm::vec3 edge2_twinnormal = edge2->m_twin->m_face->m_plane;
			// Check if the two edges build a minkowski face
			if (test_gaussmap_intersect(edge1_normal, edge1_twinnormal, -edge2_normal, -edge2_twinnormal, -edge1_dir, -edge2_dir))
			{
				// Compute centroide of A in space of B 
				const glm::vec3 centroidA = tr_point(trAtoB, glm::vec3{ 0.0f });
				// Compute edge penetration
				const float penetration = compute_edge_penetration(edge1_start, edge2_start, centroidA, edge1_dir, edge2_dir);
				// If penetration is negative -> Found a separating axis
				if (penetration < 0.0f)
				{
					penetration_data edge_pen{ actor::Edge, penetration };
					edge_pen.m_edgeA = edge1;	
					edge_pen.m_edgeB = edge2;
					return edge_pen;
				}
				// Store minimum penetration
				if (penetration < min_penetration.m_penetration)
				{
					min_penetration.m_edgeA = edge1;
					min_penetration.m_edgeB = edge2;
					min_penetration.m_penetration = penetration;
					// Store edge data to avoid
					// unnecesary recomputations
					edge_data[0] = edge1_start;
					edge_data[1] = edge1_end;
					edge_data[2] = edge2_start;
					edge_data[3] = edge2_end;
				}
			}
		}
	}
	// Return minimum penetration
	return min_penetration;
}

bool sat::test_gaussmap_intersect(const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, const glm::vec3 & d, const glm::vec3 & bXa, const glm::vec3 & dXc) const
{
	// Precompute products
	const float cba = glm::dot(c, bXa);
	const float dba = glm::dot(d, bXa);
	const float adc = glm::dot(a, dXc);
	const float bdc = glm::dot(b, dXc);
	return // Intersection tests
		   cba * dba < 0.0f
		&& adc * bdc < 0.0f
		   // Hemisphere test	
		&& cba * bdc > 0.0f;
}

float sat::compute_edge_penetration(const glm::vec3 & edge1_start, const glm::vec3 & edge2_start, const glm::vec3 & centroidA, const glm::vec3 & edge1_dir, const glm::vec3 & edge2_dir)
{
	// Compute penetration axis
	glm::vec3 axis = glm::cross(edge1_dir, edge2_dir);
	// Check vectors are parallel
	if (glm::length(axis) < c_epsilon)
		return FLT_MAX;
	// Check axis is going from A to B
	axis = glm::normalize(axis);
	if (glm::dot(axis, centroidA - edge1_start) > 0.0f)
		axis = -axis;
	// Compute distance between edges
	return glm::dot(-axis, edge2_start - edge1_start);
}

sat::result sat::generate_manifold(const penetration_data & data)
{
	// If we have an edge vs edge
	if (data.m_actor == actor::Edge)
	{
		// Get edge info
		const glm::vec3 edge1_start{ edge_data[0] };
		const glm::vec3 edge1_end{ edge_data[1] };
		const glm::vec3 edge2_start{ edge_data[2] };
		const glm::vec3 edge2_end{ edge_data[3] };
		// Compute closest point between edges
		const auto closest = closest_point_segments(edge1_start, edge1_end, edge2_start, edge2_end);
		const glm::vec3& edge1_closest = closest.first;
		const glm::vec3& edge2_closest = closest.second;
		// Get closest in local A
		const glm::vec3 edge1_closestlocal = tr_point(trBtoA, edge1_closest);
		// Compute normal in A
		const glm::vec3 normalA = edge1_closestlocal;
		assert(glm::length2(normalA) > 0.0f);
		// Compute normal in world
		const glm::vec3 normalW = glm::normalize(tr_vector(trAtoWorld, normalA));
		assert(glm::length2(normalW) > 0.0f);
		// Create contact manifold
		simple_manifold manifold;
		manifold.m_normal = normalW;
		manifold.m_local_A = { edge1_closestlocal };
		manifold.m_local_B = { edge2_closest };
		return {true, manifold };
	}
	// If the axis is a face normal
	else
	{
		// Get Reference/Incident data
		bool actor_is_A{ data.m_actor == actor::A };
		const physical_mesh* mRef{ actor_is_A ? mA : mB };
		const physical_mesh* mInc{ actor_is_A ? mB : mA };
		const glm::mat4& trRefToInc{ actor_is_A ? trAtoB : trBtoA };
		const glm::mat4& trIncToRef{ actor_is_A ? trBtoA : trAtoB };
		// Compute axis in local A & local B
		const face* faceRef = data.m_face;
		const glm::vec3 axisRef = faceRef->m_plane;
		const glm::vec3 axisInc = tr_vector(trRefToInc, axisRef);
		// Compute normal in world
		const glm::vec3 normalW = actor_is_A ? tr_vector(trAtoWorld, axisRef) : -tr_vector(trBtoWorld, axisRef);
		// Find most antiparrallel face
		const face* faceInc = mInc->find_most_antiparallel_face(axisInc);
		// Fill face vertices in Reference space
		std::vector<glm::vec3> verticesInc;
		for (uint idx : faceInc->m_indices)
		{
			const glm::vec3 v = mInc->m_vertices[idx];
			verticesInc.push_back(tr_point(trIncToRef, v));
		}
		// Fill clip planes in Reference space
		std::vector<std::pair<glm::vec3,glm::vec3> > clipPlanes;
		const half_edge* cur_hedge = faceRef->m_hedge_start;
		const half_edge* hedge_start = cur_hedge;
		do
		{
			const glm::vec3 edgeA = mRef->m_vertices[cur_hedge->get_start()];
			const glm::vec3 edgeB = mRef->m_vertices[cur_hedge->get_end()];
			const glm::vec3 edge_dir = edgeB - edgeA;
			const glm::vec3 clip_plane_normal= glm::normalize(glm::cross(axisRef, edge_dir));
			clipPlanes.push_back({ clip_plane_normal, edgeA });
			cur_hedge = cur_hedge->m_next;

		} while (cur_hedge != hedge_start);
		// Clip vertices
		std::vector<glm::vec3> clipVertices = clip(verticesInc, clipPlanes);
		// Find any point in reference
		const glm::vec3 vtxRef = mRef->m_vertices[faceRef->m_indices[0]];
		// Create contact manifold
		simple_manifold manifold;
		// Set normal
		manifold.m_normal = normalW;
		// Check each clipped vertex
		for (auto v : clipVertices)
		{
			// Compute vertex penetration
			const float penetration = glm::dot(vtxRef - v, axisRef);
			// If penetration is positive -> push point as contact
			if (penetration >= 0.0f)
			{
				// Compute local_A & local_B
				const glm::vec3 pointInc = tr_point(trRefToInc, v);
				const glm::vec3 pointRef = project_point_plane(v, axisRef, vtxRef);
				// Add points to manifold
				manifold.m_local_A.push_back(actor_is_A ? pointRef : pointInc);
				manifold.m_local_B.push_back(actor_is_A ? pointInc : pointRef);
			}
		}
		// Check if every point is clipped (corned case)
		if (manifold.m_local_A.size() == 0u)
			return {};
		// Return manifold 
		return { true,manifold };
	}
}

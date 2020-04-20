#include "contact_info.h"
#include "math_utils.h"
#include "body.h"

overlap_pair::overlap_pair(body * bA, body * bB, const physical_mesh * mA, const physical_mesh * mB)
	:body_A(bA),body_B(bB), mesh_A(mA), mesh_B(mB)
{
	manifold.oldvec_U = glm::zero<glm::vec3>();
	manifold.oldvec_V = glm::zero<glm::vec3>();
}

void overlap_pair::update()
{
	manifold.coef_friction = std::sqrtf(body_A->m_friction_coef * body_B->m_friction_coef);
	manifold.coef_roll = std::fmax(body_A->m_roll_coef, body_B->m_roll_coef);
	manifold.coef_restitution = std::sqrtf(body_A->m_restitution_coef * body_B->m_restitution_coef);

	manifold.invM_A = body_A->get_invmass();
	manifold.invM_B = body_B->get_invmass();
	manifold.invI_A = body_A->get_oriented_invinertia();
	manifold.invI_B = body_B->get_oriented_invinertia();
	// Get data
	const glm::vec3 normal = manifold.normal;
	const glm::vec3 Pos_A = body_A->m_position;
	const glm::vec3 Pos_B = body_B->m_position;
	// Get transformation matrices
	const glm::mat4 AtoW = body_A->get_model();
	const glm::mat4 BtoW = body_B->get_model();
	// Accumulate friction point
	manifold.avg_point_A =glm::zero<glm::vec3>();
	manifold.avg_point_B =glm::zero<glm::vec3>();


	// Update contact points
	for (auto& p : manifold.points)
	{
		// Compute world position
		p.point_A = tr_point(AtoW, p.local_A);
		p.point_B = tr_point(BtoW, p.local_B);
		p.depth = glm::distance(p.point_A, p.point_B);
		// Compute R vectors
		const glm::vec3 R_A = p.point_A - Pos_A;
		const glm::vec3 R_B = p.point_B - Pos_B;
		// Accumulate average points
		manifold.avg_point_A += p.point_A;
		manifold.avg_point_B += p.point_B;


		// Compute inverse mass of velocity constraint
		const glm::vec3 rAxN = glm::cross(R_A, normal);
		const glm::vec3 rBxN = glm::cross(R_B, normal);
		const float eff_mass
			= manifold.invM_A
			+ manifold.invM_B
			+ glm::dot(glm::cross(manifold.invI_A * rAxN, R_A), normal)
			+ glm::dot(glm::cross(manifold.invI_B * rBxN, R_B), normal);
		p.invM_Vel = eff_mass > 0.0f ? 1.f / (eff_mass * (float)manifold.points.size()) : 0.0f;


		// Compute velocities at contact points
		const glm::vec3 vpA = body_A->get_velocity_at_point(p.point_A);
		const glm::vec3 vpB = body_B->get_velocity_at_point(p.point_B);
		// Compute initial Jv_vel at contact point
		const float Jv0_vel = glm::dot(vpB- vpA, normal);


		// Compute restitution bias
		p.restitution_bias = 0.0f;
		if (Jv0_vel < -c_rest_vel_threshold)
			p.restitution_bias = Jv0_vel * manifold.coef_restitution;
	}


	// Compute average points
	float inv_c = 1.0f / static_cast<float>(manifold.points.size());
	manifold.avg_point_A *= inv_c;
	manifold.avg_point_B *= inv_c;
	// Compute R vectors
	const glm::vec3 R_A = manifold.avg_point_A - Pos_A;
	const glm::vec3 R_B = manifold.avg_point_B - Pos_B;


	// Prepare data for warm start
	manifold.oldvec_U = manifold.vec_U;
	manifold.oldvec_V = manifold.vec_V;


	// Compute friction vectors
	const glm::vec3 vpA = body_A->get_velocity_at_point(manifold.avg_point_A);
	const glm::vec3 vpB = body_B->get_velocity_at_point(manifold.avg_point_B);
	// Compute difference vectors
	const glm::vec3 delta_vp = vpB - vpA;
	// Compute tangent velocity
	const glm::vec3 normal_velocity = delta_vp * normal * normal;
	const glm::vec3 tangent_velocity = delta_vp - normal_velocity;
	// If tangent is valid use it as first constraint vector for consistency
	if (glm::length2(tangent_velocity) > c_epsilon)
		manifold.vec_U = glm::normalize(tangent_velocity);
	// Otherwise get a vector orthogonal to the normal
	else 
		manifold.vec_U = make_ortho(normal);
	// Compute second constraint vector
	manifold.vec_V = glm::normalize(glm::cross(normal, manifold.vec_U));


	// Compute inverse mass of first friction constraint
	const glm::vec3 rAxU = glm::cross(R_A, manifold.vec_U);
	const glm::vec3 rBxU = glm::cross(R_B, manifold.vec_U);
	const float M_U
		= manifold.invM_A
		+ manifold.invM_B
		+ glm::dot(glm::cross(manifold.invI_A*rAxU, R_A), manifold.vec_U)
		+ glm::dot(glm::cross(manifold.invI_B*rBxU, R_B), manifold.vec_U);
	manifold.invM_U = M_U > 0.0f ? 1.f / M_U : 0.0f;


	// Compute inverse mass of second friction constraint
	const glm::vec3 rAxV = glm::cross(R_A, manifold.vec_V);
	const glm::vec3 rBxV = glm::cross(R_B, manifold.vec_V);
	const float M_V
		= manifold.invM_A
		+ manifold.invM_B
		+ glm::dot(glm::cross(manifold.invI_A*rAxV, R_A), manifold.vec_V)
		+ glm::dot(glm::cross(manifold.invI_B*rBxV, R_B), manifold.vec_V);
	manifold.invM_V = M_V > 0.0f ? 1.f / M_V : 0.0f;


	// Compute inverse mass of twist friction constraint
	const float M_Twist
		= glm::dot(normal, manifold.invI_A*normal)
		+ glm::dot(normal, manifold.invI_B*normal);
	manifold.invM_Twist = M_Twist > 0.0f ? 1.f / M_Twist : 0.0f;


	// Compute inverse mass of roll resistance constraint
	manifold.invM_Roll = glm::mat3(0.0f);
	if (manifold.coef_roll > 0.0f)
	{
		manifold.invM_Roll = manifold.invI_A + manifold.invI_B;

		if (glm::determinant(manifold.invM_Roll) != 0.0f)
			manifold.invM_Roll = glm::inverse(manifold.invM_Roll);
		else
			manifold.invM_Roll = glm::mat3(0.0f);
	}
}

void overlap_pair::add_manifold(const sat::simple_manifold & other)
{
	assert(other.m_local_A.size() == other.m_local_B.size());

	// Check if both manifold refer to the same normal
	// If have the same normal -> update points
	if (glm::dot(manifold.normal, other.m_normal) > 1.0f - c_epsilon)
	{

		std::vector<contact_point> points;
		points.reserve(other.m_local_A.size());
		for (int i = 0; i < other.m_local_A.size(); ++i)
		{
			const glm::vec3& local_A = other.m_local_A[i];
			const glm::vec3& local_B = other.m_local_B[i];

			// Check wether the point is already inside
			bool found{ false };
			for (const auto prev_p : manifold.points)
				if (glm::length2(local_A - prev_p.local_A) < c_epsilon
				 && glm::length2(local_B - prev_p.local_B) < c_epsilon)
				{
					points.push_back(prev_p);
					found = true;
					break;
				}

			// Insert a new point
			if (!found)
				points.emplace_back(contact_point{ local_A, local_B });
		}
		manifold.points = points;
	}

	// If the normal is different -> overwrite manifold
	else
	{
		manifold.normal = other.m_normal;
		manifold.points.clear();
		manifold.points.reserve(other.m_local_A.size());
		for (int i = 0; i < other.m_local_A.size(); ++i)
			manifold.points.emplace_back(contact_point{ other.m_local_A[i],other.m_local_B[i] });

		manifold.lambda_U = 0.0f;
		manifold.lambda_V = 0.0f;
		manifold.lambda_Twist = 0.0f ;
		manifold.lambda_Roll = glm::vec3{ 0.0f, 0.0f, 0.0f };
	}
}

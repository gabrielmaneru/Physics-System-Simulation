#include "contact_solver.h"
#include "body.h"
#include "math_utils.h"

void constraint_contact_solver::evaluate(std::vector<overlap_pair*>& overlaps)
{
	// Warm start from previous lambdas
	for (auto& pair : overlaps)
	{
		contact_manifold& manifold = pair->manifold;
		bool resting_contact{ false };
		for (auto& p : pair->manifold.points)
		{
			// Apply previous linear impulses
			if (m_warm_start && p.lambda_Vel > 0.0f)
			{
				// At least one resting contact
				resting_contact = true;
			
			
				// Apply previous impulse
				const glm::vec3 dir_impulse = p.lambda_Vel * manifold.normal;
				pair->body_A->add_impulse(-dir_impulse, p.point_A);
				pair->body_B->add_impulse(dir_impulse, p.point_B);
			}
			// Reset previous lambdas
			else
				p.lambda_Vel = 0.0f;
		}

		// Apply previous friction & roll impulses
		if (m_warm_start && resting_contact)
		{
			// Compute previous impulse
			const glm::vec3 old_friction_impulse
				= manifold.lambda_U * manifold.oldvec_U
				+ manifold.lambda_V * manifold.oldvec_V;
			// Project previous impulse on new friction vectors
			manifold.lambda_U = glm::dot(old_friction_impulse, manifold.vec_U);
			manifold.lambda_V = glm::dot(old_friction_impulse, manifold.vec_V);
			// Compute previous impulse on new friction vectors
			const glm::vec3 friction_impulse
				= manifold.lambda_U * manifold.vec_U
				+ manifold.lambda_V * manifold.vec_V;
			// Apply previous impulse
			pair->body_A->add_impulse(-friction_impulse, manifold.avg_point_A);
			pair->body_B->add_impulse(friction_impulse, manifold.avg_point_B);


			// Compute previous twist impulse
			const glm::vec3 friction_impulsetwist = manifold.lambda_Twist * manifold.normal;
			// Apply previous twist impulse
			pair->body_A->add_impulse_angular(-friction_impulsetwist);
			pair->body_B->add_impulse_angular(friction_impulsetwist);


			// Apply previous roll impulse
			pair->body_A->add_impulse_angular(-manifold.lambda_Roll);
			pair->body_B->add_impulse_angular(manifold.lambda_Roll);
		}
		// Reset previous lambdas
		else
		{
			manifold.lambda_U = 0.0f;
			manifold.lambda_V = 0.0f;
			manifold.lambda_Twist = 0.0f;
			manifold.lambda_Roll = glm::vec3(0.0f);
		}
	}

	// For each iteration
	for (int it = 0; it < m_iteration_count; ++it)
	{
		// For each pair of overlaps
		for (auto& pair : overlaps)
		{
			// Copy previous state data
			const body bA{ *pair->body_A };
			const body bB{ *pair->body_B };
			// Get manifold data
			contact_manifold& manifold = pair->manifold;
			const glm::vec3& n = manifold.normal;
			// Accumulate linear penetration lambdas
			float accum_lambda = 0.0;


			// For each contact point in the manifold
			for (auto& point : manifold.points)
			{
				// Compute velocities at contact points
				const glm::vec3 vpA = bA.get_velocity_at_point(point.point_A);
				const glm::vec3 vpB = bB.get_velocity_at_point(point.point_B);
				

				// Compute penetration bias
				const float extra_depth = point.depth - c_depth_threshold;
				const float penetration_bias = -m_baumgarte * extra_depth / physics_dt;
				// Compute total bias
				const float b = penetration_bias + point.restitution_bias;


				// Compute Jv of the constraint
				const float Jv_vel = glm::dot(vpB - vpA, n);

				// Store previous state
				const float old_lambda = point.lambda_Vel;
				// Compute lambda differential
				float delta_lambda = point.invM_Vel * -(Jv_vel + b);
				// Apply lambda differential
				point.lambda_Vel = glm::max(point.lambda_Vel + delta_lambda, 0.0f);
				// Compute real lambda differential
				delta_lambda = point.lambda_Vel - old_lambda;


				// Accumulate linear penetration lambdas
				accum_lambda += point.lambda_Vel;


				// Apply delta impulse
				const glm::vec3 dir_impulse = delta_lambda * n;
				pair->body_A->add_impulse(-dir_impulse, point.point_A);
				pair->body_B->add_impulse(dir_impulse, point.point_B);
			}


			// Compute velocities at friction points
			glm::vec3 vfA = bA.get_velocity_at_point(manifold.avg_point_A);
			glm::vec3 vfB = bB.get_velocity_at_point(manifold.avg_point_B);
			// Compute angular velocities
			glm::vec3 wA = bA.get_angular_velocity();
			glm::vec3 wB = bB.get_angular_velocity();
			// Compute maximum friction limit
			const float max_friction_lambda = manifold.coef_friction * accum_lambda;
			const float max_roll_lambda = manifold.coef_roll * accum_lambda;
			
			
			// Compute Jv of the constraint
			const float     Jv_u     = glm::dot(vfB - vfA, manifold.vec_U);
			const float     Jv_v     = glm::dot(vfB - vfA, manifold.vec_V);
			const float     Jv_twist = glm::dot(wB - wA, n);
			const glm::vec3 Jv_roll  = wB - wA;
			// Store previous state
			const float     old_lambda_u     = manifold.lambda_U;
			const float     old_lambda_v     = manifold.lambda_V;
			const float     old_lambda_twist = manifold.lambda_Twist;
			const glm::vec3 old_lambda_roll  = manifold.lambda_Roll;
			// Compute lambda differential
			float     delta_lambda_u     = manifold.invM_U     * -Jv_u;
			float     delta_lambda_v     = manifold.invM_V     * -Jv_v;
			float     delta_lambda_twist = manifold.invM_Twist * -Jv_twist;
			glm::vec3 delta_lambda_roll  = manifold.invM_Roll  * -Jv_roll;
			// Apply lambda differential
			manifold.lambda_U     = glm::clamp(manifold.lambda_U     + delta_lambda_u,     -max_friction_lambda, max_friction_lambda);
			manifold.lambda_V     = glm::clamp(manifold.lambda_V     + delta_lambda_v,     -max_friction_lambda, max_friction_lambda);
			manifold.lambda_Twist = glm::clamp(manifold.lambda_Twist + delta_lambda_twist, -max_friction_lambda, max_friction_lambda);
			manifold.lambda_Roll  = glm::clamp(manifold.lambda_Roll  + delta_lambda_roll,  -max_roll_lambda,     max_roll_lambda);
			// Compute real lambda differential
			delta_lambda_u     = manifold.lambda_U     - old_lambda_u;
			delta_lambda_v     = manifold.lambda_V     - old_lambda_v;
			delta_lambda_twist = manifold.lambda_Twist - old_lambda_twist;
			delta_lambda_roll  = manifold.lambda_Roll  - old_lambda_roll;
			
			
			// Compute friction impulses
			const glm::vec3 impulse_linear
				= delta_lambda_u * manifold.vec_U
				+ delta_lambda_v * manifold.vec_V;
			const glm::vec3 impulse_angular = delta_lambda_twist * n + delta_lambda_roll;
			// Apply impulses
			pair->body_A->add_impulse(-impulse_linear, manifold.avg_point_A);
			pair->body_B->add_impulse(impulse_linear, manifold.avg_point_B);
			pair->body_A->add_impulse_angular(-impulse_angular);
			pair->body_B->add_impulse_angular(impulse_angular);
		}
	}
}

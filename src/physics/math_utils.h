#pragma once
#include <glm/glm.hpp>
#include <utility>
#include <vector>

const float c_epsilon{ 1e-3f };
const float c_rest_vel_threshold{ 1.0f };
const float c_slop{ 0.00f };

glm::vec3 tr_point(glm::mat4 m, glm::vec3 v);
glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v);

std::pair<glm::vec3, glm::vec3> closest_point_segments(const glm::vec3 & seg1A, const glm::vec3 & seg1B, const glm::vec3 & seg2A, const glm::vec3 & seg2B);

float compute_seg_plane_intersection(const glm::vec3& v1, const glm::vec3& v2, float plane_d, const glm::vec3& plane_n);

std::vector<glm::vec3> clip(const std::vector<glm::vec3>& vertices, const std::vector<std::pair<glm::vec3, glm::vec3> >& clipping_planes);
glm::vec3 project_point_plane(const glm::vec3& point, const glm::vec3& normal, const glm::vec3& plane_p);
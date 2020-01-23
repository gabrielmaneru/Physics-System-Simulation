#include "body.h"

glm::mat4 body::get_model()
{
	return glm::translate(glm::mat4(1.0f), m_position) * glm::mat4_cast(m_rotation);
}

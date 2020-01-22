#include "drawer.h"
#include "shader_program.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

bool c_drawer::initialize()
{
	// Initialize GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return false;
	if (!GLAD_GL_VERSION_3_3) return false;

	// Create shaders
	try {
		color_shader = new Shader_Program("color.vert", "color.frag");
	}
	catch (const std::string & log) { std::cout << log; }



	float vertices[] = {
		-0.5f, -0.5f, 0.0f, // left  
		 0.5f, -0.5f, 0.0f, // right 
		 0.0f,  0.5f, 0.0f  // top   
	};
	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	return true;
}

void c_drawer::render()
{
	m_camera.update();
	glClearColor(0.1f, 0.1f, 0.1f, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	glm::mat4 vp = m_camera.m_proj * m_camera.m_view;
	color_shader->use();
	color_shader->set_uniform("uniform_mvp", vp);
	color_shader->set_uniform("uniform_color", glm::vec4{ 1,0,0,0 });
	glDrawArrays(GL_TRIANGLES, 0, 3);
}

void c_drawer::shutdown()
{
}

c_drawer & c_drawer::get_instance()
{
	static c_drawer instance;
	return instance;
}

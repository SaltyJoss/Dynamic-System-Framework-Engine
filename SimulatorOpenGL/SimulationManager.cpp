#include "SimulationManager.h"

const GLfloat SimulationManager::Buffers::vertices[] = {
	//	 COORDINATES	 /	   COLOURS	   //
	-0.5f, 0.0f,  0.5f,		0.75f, 0.2f, 1.0f,
	-0.5f, 0.0f, -0.5f,		1.0f, 0.1f, 0.3f,
	 0.5f, 0.0f, -0.5f,		0.1f, 0.5f, 1.0f,
	 0.5f, 0.0f,  0.5f,		0.89f, 0.43f, 0.234f,
	 0.0f, 0.8f,  0.0f,		0.90676f, 0.523f, 0.0f
};

const GLuint SimulationManager::Buffers::indices[] = {
	0, 1, 2,
	0, 2, 3,
	0, 1, 4,
	1, 2, 4,
	2, 3, 4,
	3, 0, 4
};

bool SimulationManager::Init() {
	shaderProgram = std::make_unique<Shader>("default.vert", "default.frag");

	_VAO.Bind();	// Bind the _VAO

	_VBO.Init(Buffers::vertices, sizeof(Buffers::vertices));
	_EBO.Init(Buffers::indices, sizeof(Buffers::indices));

	_VAO.LinkAttrib(_VBO, _EBO, 0, 3, GL_FLOAT, 6 * sizeof(float), (void*)0);	// Links _VAO to VBO and the attributes
	_VAO.LinkAttrib(_VBO, _EBO, 1, 3, GL_FLOAT, 6 * sizeof(float), (void*)(3 * sizeof(float)));

	_VAO.Unbind();			// Unbind the _VAO
	_VBO.Unbind();			// Unbind the VBO
	_EBO.Unbind();			// Unbind the EBO

	uniID = glGetUniformLocation(shaderProgram->ID, "scale");

	return true;
}

void SimulationManager::Update(float dt) {
	/* STUBBED FOR LATER USE*/
}

void SimulationManager::Render() {
	shaderProgram->Activate();		// Activate the shader program

	model		= glm::mat4(1.0f);	// Objects coordinates
	view		= glm::mat4(1.0f);		// Camera Coordinates
	projection  = glm::mat4(1.0f);

	view		= glm::translate(view, glm::vec3(0.0f, -0.5f, -2.0f)); // Indicates which direction and how much to move the whole "world"
	projection	= glm::perspective(glm::radians(45.0f), 1280.0f / 720.0f, 0.1f, 100.0f); // (Field of View, Aspect Ratio, Closet Point Visible, Furthest Point Visable)

	int modelLoc = glGetUniformLocation(shaderProgram->ID, "model");
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	int viewLoc = glGetUniformLocation(shaderProgram->ID, "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
	int projLoc = glGetUniformLocation(shaderProgram->ID, "projection");
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

	glUniform1f(uniID, 0.4f);		// Scale of triangles 

	_VAO.Bind();					// Bind the _VAO

	glDrawElements(GL_TRIANGLES, 9, GL_UNSIGNED_INT, 0); // Draw the triangle using the EBO
}

void SimulationManager::Terminate() {
	_VAO.Delete();
	_VBO.Delete();
	_EBO.Delete();
	shaderProgram->Delete();
}


/*
 NOTES:
 - z-axis is positive TOWARDS the object (us) and negative AWAY from the object (us) ~ does this mean its observer relative?
 - In terms of projection, if anything is closer than 0.1 units, or further away than 100 units, it will be CLIPPED (clipped coords using the projection matrix)
	. Easy visulisation of this for me is minecraft, game I have played since 2011, and have modded, so despite me making notes I understand this.
 - 
*/
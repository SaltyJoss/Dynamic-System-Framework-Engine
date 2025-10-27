#ifndef SHADER_CLASS_H
#define SHADER_CLASS_H

#include <glad/glad.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>

std::string get_file_content(const char* filename);

class Shader
{
public:
	GLuint ID;
	Shader(const char* vertexPath, const char* fragmentPath);

	void Activate();
	void Delete();
};

#endif
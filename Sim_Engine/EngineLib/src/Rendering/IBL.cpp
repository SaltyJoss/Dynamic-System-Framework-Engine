
#include "pch.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <glm/glm.hpp>

#include <stb/stb_image.h>

#include "Rendering/IBL.h"
#include "Rendering/ShaderUtil.h"
#include "Rendering/CubeVertices.h"

#include "EngineLib/LogMacros.h"

namespace render {
/*
* ----------------------------------------------
*					IBL METHODS
* ----------------------------------------------
* 
* Summary:
* ----------------------------------------------
* IBL() -> Constructor that initializes the IBL object.
* ~IBL() -> Destructor that cleans up IBL resources.
* init(const std::string& hdrPath) -> Initializes the IBL by loading the HDR image and generating the necessary maps.
* ----------------------------------------------
*/
	IBL::IBL() = default;

	IBL::~IBL() {
		if (_envCubemap)	glDeleteTextures(1, &_envCubemap);
		if (_irradianceMap)	glDeleteTextures(1, &_irradianceMap);
		if (_prefilterMap)	glDeleteTextures(1, &_prefilterMap);
		if (_brdfLUT)		glDeleteTextures(1, &_brdfLUT);
		LOG_INFO("IBL resources deleted in destructor");

	}

	void IBL::init(const std::string& hdrPath) {
		loadHDR(hdrPath);
		generateCubemap();
		generateIrradianceMap();
		generatePrefilterMap();
		generateBRDFLUT();

		D_SUCCESS("IBL built successfully.");
	}

/*
 * ----------------------------------------------
 *				   HDR LOADING
 * ----------------------------------------------
 * 
 * Summary:
 * ----------------------------------------------
 * loadHDR() -> Loads an HDR image from the specified file path and creates an OpenGL texture for it.
 * ----------------------------------------------
 */

	void IBL::loadHDR(const std::string& hdrPath) {
		stbi_set_flip_vertically_on_load(true);
		int width, height, nrComponents;
		float* data = stbi_loadf(hdrPath.c_str(), &width, &height, &nrComponents, 0);

		if(!data) {
			LOG_ERROR("Failed to load HDR image from %s", hdrPath.c_str());
			D_FAIL("Failed to load HDR image from %s", hdrPath.c_str());
			return;
		}

		glGenTextures(1, &_hdrTexture);
		glBindTexture(GL_TEXTURE_2D, _hdrTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);

		LOG_INFO("HDR image loaded from %s", hdrPath.c_str(), width, height);
	}

/*
 * ----------------------------------------------
 *		EQUIRECTANGULAR TO CUBEMAP CONVERSION
 * ----------------------------------------------
 * 
 * Summary:
 * ----------------------------------------------
 * generateCubemap() -> Converts the loaded equirectangular HDR texture into a cubemap texture for environment mapping.
 * ----------------------------------------------
 */

	void IBL::generateCubemap() {
		glGenTextures(1, &_envCubemap);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);

		const unsigned int CUBE_RES = 2048;
		for (unsigned int i = 0; i < 6; ++i) {
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, CUBE_RES, CUBE_RES, 0, GL_RGB, GL_FLOAT, nullptr);
		}

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		GLuint captureFBO, captureRBO;
		glGenFramebuffers(1, &captureFBO);
		glGenRenderbuffers(1, &captureRBO);

		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
		glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, CUBE_RES, CUBE_RES);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, captureRBO);
		
		GLuint cubeVAO, cubeVBO;
		glGenVertexArrays(1, &cubeVAO);
		glGenBuffers(1, &cubeVBO);
		glBindVertexArray(cubeVAO);
		glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_VERTICES), CUBE_VERTICES, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		
		glm::mat4 captureProj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
		glm::mat4 captureViews[] = {
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f))
		};

		shaders::Shader eqShader;
		eqShader.load("Engine/assets/shaders/cubemap.vert.glsl", "Engine/assets/shaders/equirect_to_cubemap.frag.glsl");
		eqShader.use();
		eqShader.setInt1(0, "equirectMap");
		eqShader.setMat4(captureProj, "projection");

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, _hdrTexture);

		glViewport(0, 0, CUBE_RES, CUBE_RES);
		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);

		for (unsigned int i = 0; i < 6; ++i) {
			eqShader.setMat4(captureViews[i], "view");
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, _envCubemap, 0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glBindVertexArray(cubeVAO);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);
		glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

		glDeleteVertexArrays(1, &cubeVAO);
		glDeleteBuffers(1, &cubeVBO);
		glDeleteFramebuffers(1, &captureFBO);
		glDeleteRenderbuffers(1, &captureRBO);

		LOG_INFO("Generating environment cubemap from HDR texture");
	}

/*
 * ----------------------------------------------
 *		   IRRADIANCE MAP GENERATION
 * ----------------------------------------------
 * 
 * Summary:
 * ----------------------------------------------
 * generateIrradianceMap() -> Generates the irradiance cubemap used for diffuse IBL by convolving the environment cubemap.
 * ----------------------------------------------
 */

	void IBL::generateIrradianceMap() {
		const unsigned int IRR_RES = 32;

		glGenTextures(1, &_irradianceMap);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _irradianceMap);

		for (unsigned int i = 0; i < 6; ++i) {
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, IRR_RES, IRR_RES, 0, GL_RGB, GL_FLOAT, nullptr);
		}

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		GLuint captureFBO, captureRBO;
		glGenFramebuffers(1, &captureFBO);
		glGenRenderbuffers(1, &captureRBO);

		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
		glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, IRR_RES, IRR_RES);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, captureRBO);

		GLuint cubeVAO, cubeVBO;
		glGenVertexArrays(1, &cubeVAO);
		glGenBuffers(1, &cubeVBO);
		glBindVertexArray(cubeVAO);
		glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_VERTICES), CUBE_VERTICES, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

		glm::mat4 captureProj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
		glm::mat4 captureViews[] = {
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f))
		};

		shaders::Shader irrShader;
		irrShader.load("Engine/assets/shaders/irradiance.vert.glsl", "Engine/assets/shaders/irradiance_convolution.frag.glsl");
		irrShader.use();
		irrShader.setInt1(0, "equirectMap");
		irrShader.setMat4(captureProj, "projection");

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);
		
		glViewport(0, 0, IRR_RES, IRR_RES);
		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
		for (unsigned int i = 0; i < 6; ++i) {
			irrShader.setMat4(captureViews[i], "view");
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, _irradianceMap, 0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glBindVertexArray(cubeVAO);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glDeleteVertexArrays(1, &cubeVAO);
		glDeleteBuffers(1, &cubeVBO);
		glDeleteFramebuffers(1, &captureFBO);
		glDeleteRenderbuffers(1, &captureRBO);

		LOG_INFO("Generating irradiance map");
	}

/*
 * ----------------------------------------------
 *			PREFILTER MAP GENERATION
 * ----------------------------------------------
 * 
 * Summary:
 * ----------------------------------------------
 * generatePrefilterMap() -> Generates the prefilter cubemap used for specular IBL by convolving the environment cubemap at different roughness levels and storing them in mipmap levels.
 * ----------------------------------------------
 */

	void IBL::generatePrefilterMap() {
		const unsigned int PREFILTER_RES = 128;

		glGenTextures(1, &_prefilterMap);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _prefilterMap);
		for (unsigned int i = 0; i < 6; ++i) {
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, PREFILTER_RES, PREFILTER_RES, 0, GL_RGB, GL_FLOAT, nullptr);
		}

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		
		glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

		GLuint captureFBO, captureRBO;
		glGenFramebuffers(1, &captureFBO);
		glGenRenderbuffers(1, &captureRBO);
		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
		glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
		
		GLuint cubeVAO, cubeVBO;
		glGenVertexArrays(1, &cubeVAO);
		glGenBuffers(1, &cubeVBO);
		glBindVertexArray(cubeVAO);
		glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_VERTICES), CUBE_VERTICES, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		
		glm::mat4 captureProj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
		glm::mat4 captureViews[] = {
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
			glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f))
		};

		shaders::Shader prefilterShader;
		prefilterShader.load("Engine/assets/shaders/cubemap.vert.glsl", "Engine/assets/shaders/prefilter_cubemap.frag.glsl");
		prefilterShader.use();
		prefilterShader.setInt1(0, "environmentMap");
		prefilterShader.setMat4(captureProj, "projection");

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);

		const unsigned int MAX_MIP_LEVELS = 5;
		for (unsigned int mip = 0; mip < MAX_MIP_LEVELS; ++mip) {
			unsigned int mipWidth = PREFILTER_RES * std::pow(0.5, mip);
			unsigned int mipHeight = PREFILTER_RES * std::pow(0.5, mip);
			
			glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
			glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, mipWidth, mipHeight);
			glViewport(0, 0, mipWidth, mipHeight);
			
			float roughness = (float)mip / (float)(MAX_MIP_LEVELS - 1);
			prefilterShader.setFlt1(roughness, "roughness");
			
			for (unsigned int i = 0; i < 6; ++i) {
				prefilterShader.setMat4(captureViews[i], "view");
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, _prefilterMap, mip);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glBindVertexArray(cubeVAO);
				glDrawArrays(GL_TRIANGLES, 0, 36);
			}
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glDeleteVertexArrays(1, &cubeVAO);
		glDeleteBuffers(1, &cubeVBO);
		glDeleteFramebuffers(1, &captureFBO);
		glDeleteRenderbuffers(1, &captureRBO);

		LOG_INFO("Generating prefilter map");
	}

/*
 * ----------------------------------------------
 *			  BRDF LUT GENERATION
 * ----------------------------------------------
 * 
 * Summary:
 * ----------------------------------------------
 * generateBRDFLUT() -> Generates the BRDF lookup texture used for specular IBL.
 * ----------------------------------------------
 */

	void IBL::generateBRDFLUT() {
		const unsigned int BRDF_LUT_RES = 512;

		glGenTextures(1, &_brdfLUT);
		glBindTexture(GL_TEXTURE_2D, _brdfLUT);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, BRDF_LUT_RES, BRDF_LUT_RES, 0, GL_RG, GL_FLOAT, 0);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		GLuint captureFBO, captureRBO;
		glGenFramebuffers(1, &captureFBO);
		glGenRenderbuffers(1, &captureRBO);

		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
		glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, BRDF_LUT_RES, BRDF_LUT_RES);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, captureRBO);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _brdfLUT, 0);

		GLuint quadVAO, quadVBO;
		glGenVertexArrays(1, &quadVAO);
		glGenBuffers(1, &quadVBO);
		glBindVertexArray(quadVAO);
		glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(QUAD_VERTICES), &QUAD_VERTICES, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

		shaders::Shader brdfShader;
		brdfShader.load("Engine/assets/shaders/brdf_lut.vert.glsl", "Engine/assets/shaders/brdf_lut.frag.glsl");
		brdfShader.use();

		glViewport(0, 0, BRDF_LUT_RES, BRDF_LUT_RES);
		glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glBindVertexArray(quadVAO);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glDeleteVertexArrays(1, &quadVAO);
		glDeleteBuffers(1, &quadVBO);
		glDeleteFramebuffers(1, &captureFBO);
		glDeleteRenderbuffers(1, &captureRBO);

		LOG_INFO("Generating BRDF LUT");
	}

/*
 * ----------------------------------------------
 *					GETTERS
 * ----------------------------------------------
 * 
 * Summary:
 * ----------------------------------------------
 * getIrradianceMap() -> Returns the OpenGL texture ID of the irradiance map.
 * getPrefilterMap() -> Returns the OpenGL texture ID of the prefilter map.
 * getBRDFLUT() -> Returns the OpenGL texture ID of the BRDF LUT.
 * ----------------------------------------------
 */

	GLuint IBL::getIrradianceMap() const { return _irradianceMap; }
	GLuint IBL::getPrefilterMap() const { return _prefilterMap; }
	GLuint IBL::getBRDFLUT() const { return _brdfLUT; }
}
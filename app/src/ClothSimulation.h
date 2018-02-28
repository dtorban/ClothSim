#include "MassSpringSystem.h"
#include "Integration.h"
#include "glm/glm.hpp"
#include <vector>

#include "glm/glm.hpp"
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include "OpenGL.h"
#include <api/VRGraphicsState.h>

class ClothContext;
class ClothRenderer;

class ClothSimulation : public MassSpringSystem {
	friend class ClothContext;
	friend class ClothRenderer;
public:
	ClothSimulation(Integrator& integrator);
	virtual ~ClothSimulation();

	void step(double dt);

private:
	std::vector<glm::vec3> calculateNormals(const std::vector<unsigned int>& inds, const std::vector<glm::vec3>& positions);

	Integrator& integrator;
	void* integratorMemory;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> colors;
    glm::mat4 model;
};

    class ClothContext {
    public:
        ClothContext(const ClothSimulation& cloth, const MinVR::VRGraphicsState &renderState) : cloth(cloth) {
                glewExperimental = GL_TRUE;
                GLenum err = glewInit();
                if (GLEW_OK != err)
                {
                    std::cout << "Error initializing GLEW." << std::endl;
                }

                glGenBuffers(1, &elementBuffer);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, cloth.indices.size() * sizeof(unsigned int), &cloth.indices[0], GL_STATIC_DRAW);

                // Allocate space and send Vertex Data
                glGenBuffers(1, &vbo);
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*(cloth.getPositions().size() + cloth.normals.size() + cloth.colors.size()), 0, GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ARRAY_BUFFER, 0, 3*sizeof(float)*(cloth.getPositions().size()), &cloth.getPositions()[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*cloth.getPositions().size(), 3*sizeof(float)*cloth.normals.size(), &cloth.normals[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*(cloth.getPositions().size() + cloth.normals.size()), 3*sizeof(float)*cloth.colors.size(), &cloth.colors[0]);
        }


        ~ClothContext() {
                glDeleteBuffers(1, &vbo);
        }

        void update(const MinVR::VRGraphicsState &renderState) {
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferSubData(GL_ARRAY_BUFFER, 0, 3*sizeof(float)*(cloth.getPositions().size()), &cloth.getPositions()[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*cloth.getPositions().size(), 3*sizeof(float)*cloth.normals.size(), &cloth.normals[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*(cloth.getPositions().size() + cloth.normals.size()), 3*sizeof(float)*cloth.colors.size(), &cloth.colors[0]);
        }

        GLuint getVbo() { return vbo; }
        GLuint getElementBuffer() { return elementBuffer; }

    private:
        const ClothSimulation& cloth;
        GLuint vbo;
        GLuint elementBuffer;
    };

class ClothRenderer {
    public:
        ClothRenderer(const ClothSimulation& cloth, ClothContext& sharedContext, const MinVR::VRGraphicsState &renderState) : cloth(cloth), sharedContext(sharedContext) {   
                // Init GL
                glEnable(GL_DEPTH_TEST);
                glClearDepth(1.0f);
                glDepthFunc(GL_LEQUAL);
                glClearColor(0, 0, 0, 1);

                // Create vao
                glGenVertexArrays(1, &vao);
                glBindVertexArray(vao);
                glBindBuffer(GL_ARRAY_BUFFER, sharedContext.getVbo());
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sharedContext.getElementBuffer());
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (char*)0);
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (char*)0 + 3*sizeof(float)*cloth.getPositions().size());
                glEnableVertexAttribArray(2);
                glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (char*)0 + 3*sizeof(float)*(cloth.getPositions().size() + cloth.normals.size()));

                // Create shader
                std::string vertexShader =
                        "#version 330 \n"
                        "layout(location = 0) in vec3 position; "
                        "layout(location = 1) in vec3 normal; "
                        "layout(location = 2) in vec3 color; "
                        ""
                        "uniform mat4 ProjectionMatrix; "
                        "uniform mat4 ViewMatrix; "
                        "uniform mat4 ModelMatrix; "
                        "uniform mat4 NormalMatrix; "
                        ""
                        "out vec3 col;"
                        "out vec3 norm;"
                        ""
                        "void main() { "
                        "   gl_Position = ProjectionMatrix*ViewMatrix*ModelMatrix*vec4(position, 1.0); "
                        "   col = color;"
                        "   norm = normal;"
                        "}";
                vshader = compileShader(vertexShader, GL_VERTEX_SHADER);

                std::string fragmentShader =
                        "#version 330 \n"
                        "in vec3 col;"
                        "in vec3 norm;"
                        "out vec4 colorOut;"
                        ""
                        "void main() { "
                        "   colorOut = vec4((norm/2.0) + 0.5, 1.0);"
                        "}";
                fshader = compileShader(fragmentShader, GL_FRAGMENT_SHADER);

                // Create shader program
                shaderProgram = glCreateProgram();
                glAttachShader(shaderProgram, vshader);
                glAttachShader(shaderProgram, fshader);
                linkShaderProgram(shaderProgram);
        }

        virtual ~ClothRenderer() {
                glDeleteVertexArrays(1, &vao);
                glDetachShader(shaderProgram, vshader);
                glDetachShader(shaderProgram, fshader);
                glDeleteShader(vshader);
                glDeleteShader(fshader);
                glDeleteProgram(shaderProgram);
        }

        void update(const MinVR::VRGraphicsState &renderState) {
        }

        void render(const MinVR::VRGraphicsState &renderState) {
            const float* projMat = renderState.getProjectionMatrix();
            const float* viewMat = renderState.getViewMatrix();

            // clear screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

            // Set shader parameters
            glUseProgram(shaderProgram);
            GLint loc = glGetUniformLocation(shaderProgram, "ProjectionMatrix");
            glUniformMatrix4fv(loc, 1, GL_FALSE, projMat);
            loc = glGetUniformLocation(shaderProgram, "ViewMatrix");
            glUniformMatrix4fv(loc, 1, GL_FALSE, viewMat);
            loc = glGetUniformLocation(shaderProgram, "ModelMatrix");
            glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(cloth.model));

            // Draw cube
            glBindVertexArray(vao);
            glDrawElements(GL_TRIANGLES, cloth.indices.size(), GL_UNSIGNED_INT, (void*)0);
            //glDrawElements(GL_POINTS, cloth..indices.size(), GL_UNSIGNED_INT, (void*)0);
            glBindVertexArray(0);

            // reset program
            glUseProgram(0);
        }

        /// Compiles shader
        GLuint compileShader(const std::string& shaderText, GLuint shaderType) {
            const char* source = shaderText.c_str();
            int length = (int)shaderText.size();
            GLuint shader = glCreateShader(shaderType);
            glShaderSource(shader, 1, &source, &length);
            glCompileShader(shader);
            GLint status;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
            if(status == GL_FALSE) {
                GLint length;
                glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
                std::vector<char> log(length);
                glGetShaderInfoLog(shader, length, &length, &log[0]);
                std::cerr << &log[0];
            }

            return shader;
        }

        /// links shader program
        void linkShaderProgram(GLuint shaderProgram) {
            glLinkProgram(shaderProgram);
            GLint status;
            glGetProgramiv(shaderProgram, GL_LINK_STATUS, &status);
            if(status == GL_FALSE) {
                GLint length;
                glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &length);
                std::vector<char> log(length);
                glGetProgramInfoLog(shaderProgram, length, &length, &log[0]);
                std::cerr << "Error compiling program: " << &log[0] << std::endl;
            }
        }

    private:
        const ClothSimulation& cloth;
        GLuint vao, vshader, fshader, shaderProgram;
        const ClothContext& sharedContext;
    };
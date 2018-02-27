#include <iostream>
#include <vector>
#include "MassSpringSystem.h"
#include "OpenGL.h"
#include "glm/glm.hpp"
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "VRMultithreadedApp.h"
#include <api/VRTrackerEvent.h>
#include "Force.h"
using namespace MinVR;

/**
 * MyVRApp is an example of a modern OpenGL using VBOs, VAOs, and shaders.  MyVRApp inherits
 * from VRGraphicsApp, which allows you to override onVREvent to get input events, onRenderContext
 * to setup context sepecific objects, and onRenderScene that renders to each viewport.
 */
class MyVRApp : public VRMultithreadedApp {
public:
	MyVRApp(int argc, char** argv) : VRMultithreadedApp(argc, argv), model(1.0f), time(0.0f), dt(0.005), simTime(0.0), sphere(glm::vec3(0.5,-10.6,0.0), 0.0), sphere2(glm::vec3(0.5, -10.6, 0.0), 0.15), iterationsPerFrame(10) {
        static ExplicitEulerIntegrator explicitEulerIntegrator;
        static ExplicitEulerIntegrator semiImplicitEulerIntegrator(true);
        static RungaKutta4Integrator rungaKutta4Integrator;
        static ImplicitEulerIntegrator implicitEulerIntegrator;

        int width = 10;
        int height = 10;

		float ks = 200.0f;
		float kd = 0.1;
		float totalMass = 1.0f;

        integrator = &explicitEulerIntegrator;  dt = 0.0005; width = 30; height = 30; iterationsPerFrame = 5; ks = 500.0f; kd = 1.0f; totalMass = 20.0f;
		integrator = &semiImplicitEulerIntegrator;  dt = 0.001; width = 30; height = 30; iterationsPerFrame = 5; ks = 8000.0f; kd = 1.0f; totalMass = 20.0f;
		integrator = &rungaKutta4Integrator; dt = 0.001; width = 30; height = 30; iterationsPerFrame = 5; ks = 10000.0f; kd = 3.0f; totalMass = 20.0f;
		integrator = &implicitEulerIntegrator; dt = 0.01; width =25; height = 25; iterationsPerFrame = 1; ks = 100.0f; kd = 0.0f; totalMass = 1.0f;
        
        //glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(0,-0.5,0));
        model = glm::translate(glm::mat4(1), glm::vec3(0.5,1.5,0));
        model = glm::scale(model, glm::vec3(1.0f)*2.0f);
        glm::mat4 transform(1.0f);
        transform = glm::rotate(transform, float(-3.141519 / 2), glm::vec3(1.0, 0.0, 0.0));


        float dx = 1.0f/float(width);
        float dy = 1.0f/float(height);

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                cloth.addNode((double)totalMass/(float(width+1)*float(height+1)), glm::vec3(transform*glm::vec4(-0.5f + dx*x, -0.5f + dy*y, 0.0f,1.0f)));
               // nodes.push_back(glm::vec3(transform*glm::vec4(-0.5f + dx*x, -0.5f + dy*y, 0.0f,1.0f))); 
                //std::cout << nodes[nodes.size()-1][0] << ", " << nodes[nodes.size()-1][1] << ", " << nodes[nodes.size()-1][2] << std::endl;           
                normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
                colors.push_back(glm::vec3(0.5f, 0.5f, 0.5f));
            }
        }

        for (int x = 0; x < width-1 ; x++) {
            for (int y = 0; y < height-1; y++) {
                indices.push_back(x*height+y);
                indices.push_back(x*height+y+1);
                indices.push_back((x+1)*height+y);
                indices.push_back((x+1)*height+y);
                indices.push_back(x*height+y+1);
                indices.push_back((x+1)*height+y+1);
            }
        }

        for (int f = 0; f < indices.size(); f+=3) {
           // cloth.addForce(new AreoForce(indices[f], indices[f+1], indices[f+2], 1.0, 10.0, glm::vec3(5.5f, 0.0f, 5.0f)*1.0f, cloth.getPositions().size(), 0));
        }

        int node = 0;
        for (int x = 0; x < width ; x++) {
            for (int y = 0; y < height; y++) {
                //if (x == 2 && y == 0) {
                //if (x == 0) {
                //if ((x == 0 && y == 0) || (x == 0 && y == height-1)) {
                   //cloth.addForce(new AnchorForce(node, cloth.getPositions()[node], ks, kd, cloth.getPositions().size(), 0));
                //}

                node++;
            }
        }

        for (int x = 0; x < width-1 ; x++) {
            for (int y = 0; y < height; y++) {
                // horizontal
                cloth.addForce(new SpringForce(x*height+y, (x+1)*height+y, ks, kd, dx, cloth.getPositions().size(), 0, x == 2 && y == 0));
                if (x < (width-1)/2 && y <(height-1)/2) {
                    //cloth.addForce(new SpringForce((2*x)*height+(2*y), 2*(x+1)*height+2*y, ks/2, kd/2, dx*2, cloth.getPositions().size(), 0));
                }
            }
        }

        for (int x = 0; x < width ; x++) {
            for (int y = 0; y < height-1; y++) {
                // vertical
                cloth.addForce(new SpringForce(x*height+y, x*height+y+1, ks, kd, dy, cloth.getPositions().size(), 0));
                if (x < (width-1)/2 && y <(height-1)/2) {
                   //cloth.addForce(new SpringForce(2*x*height+2*y, 2*x*height+2*(y+1), ks/2, kd/2, dy*2, cloth.getPositions().size(), 0));   
                }
            }
        }

        // Add cross forces
        for (int x = 0; x < width-1 ; x++) {
            for (int y = 0; y < height-1; y++) {
                //cloth.addForce(new SpringForce(x*height+y, (x+1)*height+y+1, ks, kd, glm::sqrt(dx*dx+dy*dy), cloth.getPositions().size(), 0));
                //cloth.addForce(new SpringForce((x+1)*height+y, x*height+y+1, ks, kd, glm::sqrt(dx*dx+dy*dy), cloth.getPositions().size(), 0));
                if (x < (width-1)/2 && y <(height-1)/2) {
                    //cloth.addForce(new SpringForce(2*(x*height+y), 2*((x+1)*height+y+1), ks/2, kd/2, glm::sqrt(dx*dx+dy*dy)*2, cloth.getPositions().size(), 0));
                    //cloth.addForce(new SpringForce(2*((x+1)*height+y), 2*(x*height+y+1), ks/2, kd/2, glm::sqrt(dx*dx+dy*dy)*2, cloth.getPositions().size(), 0));   
                }
            }
        }

        cloth.addForce(new ConstantForce(glm::vec3(10.0,0.0,0.0), cloth.getPositions().size(), 0));
		//cloth.addCollider(&sphere);
		//cloth.addCollider(&sphere2);

        integratorMemory = integrator->allocateMemory(cloth);
    }

    ~MyVRApp() {
        integrator->freeMemory(integratorMemory);
    }


	/// onVREvent is called when a new intput event happens.
	void onVREvent(const VRDataIndex &event) {

        //event.printStructure();

		//std::cout << event.getName() << std::endl;
		VRString type = (VRString)event.getValue("EventType");

		// Set time since application began
		if (event.getName() == "FrameStart") {
            time = event.getValue("ElapsedSeconds");
			// Calculate model matrix based on time
            //time = 0.0f;
			VRMatrix4 modelMatrix = VRMatrix4::rotationX(0.5f*time);
			modelMatrix = modelMatrix * VRMatrix4::rotationY(0.5f*time);
			/*for (int f = 0; f < 16; f++) {
				model[f] = modelMatrix.getArray()[f];
			}*/
			return;
		}

		if (type == "TrackerMove") {
			VRTrackerEvent e(event);
			//glm::mat4 trackerTransform = glm::make_mat4(e.getTransform());
			if (e.getName() == "HTC_Controller_Right_Move") {
				glm::vec3 trackerPos = glm::make_vec3(e.getPos());
				//std::cout << e.getName() << " " << trackerPos[0] << " " << trackerPos[1] << " " << trackerPos[2] << " " << e.index() << std::endl;
				sphere.center = glm::inverse(model)*glm::vec4(trackerPos, 1.0f);
			}
			if (e.getName() == "HTC_Controller_Left_Move") {
				glm::vec3 trackerPos = glm::make_vec3(e.getPos());
				//std::cout << e.getName() << " " << trackerPos[0] << " " << trackerPos[1] << " " << trackerPos[2] << " " << e.index() << std::endl;
				sphere2.center = glm::inverse(model)*glm::vec4(trackerPos, 1.0f);
			}
		}

		// Quit if the escape button is pressed
		if (event.getName() == "KbdEsc_Down") {
			running = false;
		}
	}

    void update() {

        framesSinceLastFPS++;

        if (framesSinceLastFPS > 50) {
            std::cout << float(framesSinceLastFPS)/(time - lastTimeFPS) << std::endl;
            lastTimeFPS = time;
            framesSinceLastFPS = 0;
        }

        float timeDiff = time - lastTime;
        lastTime = time;
        int count = 0;
        //while (simTime + dt < time && count < 10) {
        for (int f = 0; f < iterationsPerFrame; f++) {
            //for (int f = 0; f < nodes.size(); f++) {
                //nodes[f] += glm::vec3(1.0f, 0.0f, 0.0f)*float(dt);
            //}
            //std::cout << simTime << std::endl;
            integrator->step(cloth, dt, integratorMemory);
            normals = calculateNormals(indices, cloth.getPositions());
            cloth.handleCollisions();
            simTime += dt;
            count++;
        }
        
        //sphere.center += glm::vec3(-0.1*timeDiff, 0.0, 0.0);

    }

    std::vector<glm::vec3> calculateNormals(const std::vector<unsigned int>& inds, const std::vector<glm::vec3>& positions) {
        std::vector<glm::vec3> norms(positions.size());
        std::vector<int> numNorms(positions.size());

        for (int f = 0; f < indices.size(); f+=3) {
            const glm::vec3& a = positions[inds[f]];
            const glm::vec3& b = positions[inds[f+1]];
            const glm::vec3& c = positions[inds[f+2]];
            glm::vec3 n = glm::normalize(glm::cross(b-a, c-a));
            norms[inds[f]] += n;
            norms[inds[f+1]] += n;
            norms[inds[f+2]] += n;
            numNorms[inds[f]]++;
            numNorms[inds[f+1]]++;
            numNorms[inds[f+2]]++;
        }

        for (int f = 0; f < norms.size(); f++) {
            if (numNorms[f] > 0) {
                norms[f] = norms[f]/(float(numNorms[f]));
            }
        }

        return norms;
    }

    class Context : public VRAppSharedContext {
    public:
        Context(const MyVRApp& app, const VRGraphicsState &renderState) : app(app) {
                glewExperimental = GL_TRUE;
                GLenum err = glewInit();
                if (GLEW_OK != err)
                {
                    std::cout << "Error initializing GLEW." << std::endl;
                }

                glGenBuffers(1, &elementBuffer);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, app.indices.size() * sizeof(unsigned int), &app.indices[0], GL_STATIC_DRAW);

                // Allocate space and send Vertex Data
                glGenBuffers(1, &vbo);
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*(app.cloth.getPositions().size() + app.normals.size() + app.colors.size()), 0, GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ARRAY_BUFFER, 0, 3*sizeof(float)*(app.cloth.getPositions().size()), &app.cloth.getPositions()[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*app.cloth.getPositions().size(), 3*sizeof(float)*app.normals.size(), &app.normals[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*(app.cloth.getPositions().size() + app.normals.size()), 3*sizeof(float)*app.colors.size(), &app.colors[0]);
        }

        ~Context() {
                glDeleteBuffers(1, &vbo);
        }

        void update(const VRGraphicsState &renderState) {
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferSubData(GL_ARRAY_BUFFER, 0, 3*sizeof(float)*(app.cloth.getPositions().size()), &app.cloth.getPositions()[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*app.cloth.getPositions().size(), 3*sizeof(float)*app.normals.size(), &app.normals[0]);
                glBufferSubData(GL_ARRAY_BUFFER, 3*sizeof(float)*(app.cloth.getPositions().size() + app.normals.size()), 3*sizeof(float)*app.colors.size(), &app.colors[0]);
        }

        GLuint getVbo() { return vbo; }
        GLuint getElementBuffer() { return elementBuffer; }

    private:
        const MyVRApp& app;
        GLuint vbo;
        GLuint elementBuffer;
    };

    class Renderer : public VRAppRenderer {
    public:
        Renderer(const MyVRApp& app, Context& sharedContext, const VRGraphicsState &renderState) : app(app), sharedContext(sharedContext) {   
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
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (char*)0 + 3*sizeof(float)*app.cloth.getPositions().size());
                glEnableVertexAttribArray(2);
                glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (char*)0 + 3*sizeof(float)*(app.cloth.getPositions().size() + app.normals.size()));

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

        virtual ~Renderer() {
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
            glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(app.model));

            // Draw cube
            glBindVertexArray(vao);
            glDrawElements(GL_TRIANGLES, app.indices.size(), GL_UNSIGNED_INT, (void*)0);
            //glDrawElements(GL_POINTS, app.indices.size(), GL_UNSIGNED_INT, (void*)0);
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
        const MyVRApp& app;
        GLuint vao, vshader, fshader, shaderProgram;
        const Context& sharedContext;
    };

    VRAppSharedContext* createSharedContext(const VRGraphicsState &renderState) {
        return new Context(*this, renderState);
    }

    VRAppRenderer* createRenderer(VRAppSharedContext& sharedContext, const VRGraphicsState &renderState) {
        return new Renderer(*this, *static_cast<Context*>(&sharedContext), renderState);
    }

private:
	//float model[16];
    glm::mat4 model;
    VRMain *vrMain;
    std::vector<unsigned int> indices;
    //std::vector<glm::vec3> nodes;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> colors;
    double time;
    double lastTime;
    double dt;
    double simTime;
    MassSpringSystem cloth;
    Integrator* integrator;
    void* integratorMemory;
	SphereCollider sphere;
	SphereCollider sphere2;
    float lastTimeFPS;
    int framesSinceLastFPS;
    int iterationsPerFrame;
};

/// Main method which creates and calls application
int main(int argc, char **argv) {
	MyVRApp app(argc, argv);
	app.run();
	return 0;
}

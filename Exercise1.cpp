// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <functional>

#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
// #include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <iostream>
#include <vector>

#include "shader.hpp"
#include "CamControls.hpp"
#include "Axes.hpp"
#include "TriTable.hpp"
#include <thread>

#include <queue>
#include <mutex>
#include <condition_variable>

struct face {
	glm::vec3 v1;
	glm::vec3 v2;
	glm::vec3 v3;
	face(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) : v1(v1), v2(v2), v3(v3) {}
	face() {}
};

template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable cond;
public:
    void push(T value) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(value);
        cond.notify_one();
    }

    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex);
        if(queue.empty()) {
            return false;
        }
        value = queue.front();
        queue.pop();
        return true;
    }

    void wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock, [this]{ return !queue.empty(); });
        value = queue.front();
        queue.pop();
    }

	int size() {
		return queue.size();
	}
};


float f1(float x, float y, float z) {
	return y - sin(x) * cos(z);
}

float f2(float x, float y, float z) {
	return x * x - y * y - z * z - z;
}

float f3(float x, float y, float z) {
	return x * x + y * y + z * z;
}

void marching_cubes_bonus(
	std::function<float(float, float, float)> f,
	float isovalue,
	float min,
	float max,
	float stepsize,
	ThreadSafeQueue<face>& verticesQ,
	ThreadSafeQueue<face>& normalsQ,
	const std::string& fileName)  {

	std::vector<float> vertices;
	std::vector<float> normals;
	
	float v[8];
	int* verts;
	for (float z = min; z < max; z += stepsize) {
		for (float y = min; y < max; y += stepsize) {
			for (float x = min; x < max; x += stepsize) {
				//test the cube
				v[0] = f(x, y, z);
				v[1] = f(x+stepsize, y, z);
				v[2] = f(x+stepsize, y, z+stepsize);
				v[3] = f(x, y, z+stepsize);
				v[4] = f(x, y+stepsize, z);
				v[5] = f(x+stepsize, y+stepsize, z);
				v[6] = f(x+stepsize, y+stepsize, z+stepsize);
				v[7] = f(x, y+stepsize, z+stepsize);

				int which = 0;
				for (int i = 0; i < 8; ++i) {
					if (v[i] < isovalue) {
						which |= 1 << i;
					}
				}

				verts = marching_cubes_lut[which];
				for (int i = 0; verts[i] >= 0; i += 3) {
					glm::vec3 v1 = {x+stepsize*vertTable[verts[i]][0], y+stepsize*vertTable[verts[i]][1], z+stepsize*vertTable[verts[i]][2]};
					glm::vec3 v2 = {x+stepsize*vertTable[verts[i+1]][0], y+stepsize*vertTable[verts[i+1]][1], z+stepsize*vertTable[verts[i+1]][2]};
					glm::vec3 v3 = {x+stepsize*vertTable[verts[i+2]][0], y+stepsize*vertTable[verts[i+2]][1], z+stepsize*vertTable[verts[i+2]][2]};

					vertices.emplace_back(x+stepsize*vertTable[verts[i]][0]);
					vertices.emplace_back(y+stepsize*vertTable[verts[i]][1]);
					vertices.emplace_back(z+stepsize*vertTable[verts[i]][2]);
					vertices.emplace_back(x+stepsize*vertTable[verts[i+1]][0]);
					vertices.emplace_back(y+stepsize*vertTable[verts[i+1]][1]);
					vertices.emplace_back(z+stepsize*vertTable[verts[i+1]][2]);
					vertices.emplace_back(x+stepsize*vertTable[verts[i+2]][0]);
					vertices.emplace_back(y+stepsize*vertTable[verts[i+2]][1]);
					vertices.emplace_back(z+stepsize*vertTable[verts[i+2]][2]);
				
					face vface(v1,v2,v3);
					verticesQ.push(vface);

					// Compute normals
					float Ux = v2.x - v1.x;
					float Uy = v2.y - v1.y;
					float Uz = v2.z - v1.z;
					float Vx = v3.x - v1.x;				
					float Vy = v3.y - v1.y;
					float Vz = v3.z - v1.z;

					float Nx = Uy*Vz - Uz*Vy;
					float Ny = Uz*Vx - Ux*Vz;
					float Nz = Ux*Vy - Uy*Vx;

					for (size_t j = 0; j < 9; j += 3) {
						normals.emplace_back(Nx);
						normals.emplace_back(Ny);
						normals.emplace_back(Nz);
					}

					glm::vec3 n = {Nx, Ny, Nz};

					face nface(n,n,n);
					normalsQ.push(nface);
				}
			}
		}
	}

	std::ofstream plyFile;
    plyFile.open(fileName);

    // Write the PLY header
    plyFile << "ply\n";
    plyFile << "format ascii 1.0\n";
    plyFile << "element vertex " << vertices.size() / 3 << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "property float nx\n";
    plyFile << "property float ny\n";
    plyFile << "property float nz\n";
    plyFile << "element face " << vertices.size() / 9 << "\n";
    plyFile << "property list uchar uint vertex_indices\n";
    plyFile << "end_header\n";

    // Write the vertex positions and normals
    for (size_t i = 0; i < vertices.size(); i += 3) {
        plyFile << vertices[i] << " " << vertices[i+1] << " " << vertices[i+2];
        plyFile << " " << normals[i] << " " << normals[i+1] << " " << normals[i+2] << "\n";
    }
	for (size_t i = 0; i < vertices.size() / 3; i += 3) {
        plyFile << "3 " << i << " " << i+1 << " " << i+2 << "\n";
    }

    plyFile.close();
	printf("Done exporting ply file.");
}

std::vector<float> marching_cubes(
	std::function<float(float, float, float)> f,
	float isovalue,
	float min,
	float max,
	float stepsize) {
		
	std::vector<float> vertices;

	float v[8];
	int* verts;
	for (float z = min; z < max; z += stepsize) {
		for (float y = min; y < max; y += stepsize) {
			for (float x = min; x < max; x += stepsize) {
				//test the cube
				v[0] = f(x, y, z);
				v[1] = f(x+stepsize, y, z);
				v[2] = f(x+stepsize, y, z+stepsize);
				v[3] = f(x, y, z+stepsize);
				v[4] = f(x, y+stepsize, z);
				v[5] = f(x+stepsize, y+stepsize, z);
				v[6] = f(x+stepsize, y+stepsize, z+stepsize);
				v[7] = f(x, y+stepsize, z+stepsize);

				int which = 0;
				for (int i = 0; i < 8; ++i) {
					if (v[i] < isovalue) {
						which |= 1 << i;
					}
				}

				verts = marching_cubes_lut[which];
				for (int i = 0; verts[i] >= 0; i += 3) {
					vertices.emplace_back(x+stepsize*vertTable[verts[i]][0]);
					vertices.emplace_back(y+stepsize*vertTable[verts[i]][1]);
					vertices.emplace_back(z+stepsize*vertTable[verts[i]][2]);
					vertices.emplace_back(x+stepsize*vertTable[verts[i+1]][0]);
					vertices.emplace_back(y+stepsize*vertTable[verts[i+1]][1]);
					vertices.emplace_back(z+stepsize*vertTable[verts[i+1]][2]);
					vertices.emplace_back(x+stepsize*vertTable[verts[i+2]][0]);
					vertices.emplace_back(y+stepsize*vertTable[verts[i+2]][1]);
					vertices.emplace_back(z+stepsize*vertTable[verts[i+2]][2]);
				}
			}
		}
	}

	return vertices;
}

std::vector<float> compute_normals(const std::vector<float>& vertices) {
    std::vector<float> normals(vertices.size());
    for (size_t i = 0; i < vertices.size(); i += 9) {
        // Compute the vectors representing two sides of the triangle
        float Ux = vertices[i+3] - vertices[i];
        float Uy = vertices[i+4] - vertices[i+1];
        float Uz = vertices[i+5] - vertices[i+2];
        float Vx = vertices[i+6] - vertices[i];
        float Vy = vertices[i+7] - vertices[i+1];
        float Vz = vertices[i+8] - vertices[i+2];

        // Compute the cross product of U and V
        float Nx = Uy*Vz - Uz*Vy;
        float Ny = Uz*Vx - Ux*Vz;
        float Nz = Ux*Vy - Uy*Vx;

        // Normalize the normal vector
        float length = std::sqrt(Nx*Nx + Ny*Ny + Nz*Nz);
        Nx /= length;
        Ny /= length;
        Nz /= length;

        // Assign the computed normal to each vertex of the triangle
        for (size_t j = 0; j < 9; j += 3) {
            normals[i+j] = Nx;
            normals[i+j+1] = Ny;
            normals[i+j+2] = Nz;
        }
    }
    return normals;
}

void drawBox(float min, float max) {
    glBegin(GL_LINES);

    // Bottom square
    glVertex3f(min, min, min);
    glVertex3f(max, min, min);

    glVertex3f(max, min, min);
    glVertex3f(max, max, min);

    glVertex3f(max, max, min);
    glVertex3f(min, max, min);

    glVertex3f(min, max, min);
    glVertex3f(min, min, min);

    // Top square
    glVertex3f(min, min, max);
    glVertex3f(max, min, max);

    glVertex3f(max, min, max);
    glVertex3f(max, max, max);

    glVertex3f(max, max, max);
    glVertex3f(min, max, max);

    glVertex3f(min, max, max);
    glVertex3f(min, min, max);

    // Connecting lines
    glVertex3f(min, min, min);
    glVertex3f(min, min, max);

    glVertex3f(max, min, min);
    glVertex3f(max, min, max);

    glVertex3f(max, max, min);
    glVertex3f(max, max, max);

    glVertex3f(min, max, min);
    glVertex3f(min, max, max);

    glEnd();
}

void writePLY(const std::vector<float>& vertices, const std::vector<float>& normals, const std::string& fileName) {
    std::ofstream plyFile;
    plyFile.open(fileName);

    // Write the PLY header
    plyFile << "ply\n";
    plyFile << "format ascii 1.0\n";
    plyFile << "element vertex " << vertices.size() / 3 << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "property float nx\n";
    plyFile << "property float ny\n";
    plyFile << "property float nz\n";
    plyFile << "element face " << vertices.size() / 9 << "\n";
    plyFile << "property list uchar uint vertex_indices\n";
    plyFile << "end_header\n";

    // Write the vertex positions and normals
    for (size_t i = 0; i < vertices.size(); i += 3) {
        plyFile << vertices[i] << " " << vertices[i+1] << " " << vertices[i+2];
        plyFile << " " << normals[i] << " " << normals[i+1] << " " << normals[i+2] << "\n";
    }
	for (size_t i = 0; i < vertices.size() / 3; i += 3) {
        plyFile << "3 " << i << " " << i+1 << " " << i+2 << "\n";
    }

    plyFile.close();
}

//////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[])
{

	///////////////////////////////////////////////////////
	float screenW = 1400;
	float screenH = 900;
	float stepsize = 0.05f;
	float min = -5;
	float max = 5;
	float isoval = 0;
	float function = 1;
	float bonus = 1;
	if (argc > 1 ) {
		screenW = atoi(argv[1]);
	}
	if (argc > 2) {
		screenH = atoi(argv[2]);
	}
	if (argc > 3) {
		stepsize = atof(argv[3]);
	}
	if (argc > 4) {
		min = atof(argv[4]);
	}
	if (argc > 5) {
		max = atof(argv[5]);
	}
	if (argc > 6) {
		isoval = atof(argv[6]);
	}
	if (argc > 7) {
		function = atof(argv[7]);
	}
	if (argc > 8) {
		bonus = atof(argv[8]);
	}

	///////////////////////////////////////////////////////

	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( screenW, screenH, "Marching Cubes", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Dark blue background
	glClearColor(0.2f, 0.2f, 0.3f, 0.0f);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	// Projection = glm::mat4(1.0f);
	glm::mat4 Projection = glm::perspective(glm::radians(45.0f), screenW/screenH, 0.001f, 1000.0f);
	glLoadMatrixf(glm::value_ptr(Projection));

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glm::vec3 eye = {0.0f, 3.0f, 5.0f};
	glm::vec3 up = {0.0f, 1.0f, 0.0f};
	glm::vec3 center = {0.0f, 0.0f, 0.0f};

	glm::mat4 V = glm::lookAt(eye, center, up);
	glLoadMatrixf(glm::value_ptr(V));

	glm::mat4 M(1.0f);
	
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	Axes worldaxes({min,min,min}, {max - min,max - min,max - min});

	ThreadSafeQueue<face> vertexQueue;
	ThreadSafeQueue<face> normalQueue;
	
	std::vector<float> marchingVerts;
	std::vector<float> normals;
	if(bonus == 0) {
		if(function == 1) {
			marchingVerts = marching_cubes(f1, isoval, min, max, stepsize);
			normals = compute_normals(marchingVerts);
			writePLY(marchingVerts, normals, "function1.ply");
		}
		else if(function == 2) {
			marchingVerts = marching_cubes(f2, isoval, min, max, stepsize);
			normals = compute_normals(marchingVerts);
			writePLY(marchingVerts, normals, "function2.ply");
		}
		else if(function == 3) {
			marchingVerts = marching_cubes(f3, isoval, min, max, stepsize);
			normals = compute_normals(marchingVerts);
			writePLY(marchingVerts, normals, "function3.ply");
		}
	}
	else {
		std::thread t1([&] { // Capture local variables by reference
			if(function == 1) {
				marching_cubes_bonus(f1, isoval, min, max, stepsize, vertexQueue, normalQueue, "function1.ply");
			}
			else if(function == 2) {
				marching_cubes_bonus(f2, isoval, min, max, stepsize, vertexQueue, normalQueue, "function2.ply");
			}
			else if(function == 3) {
				marching_cubes_bonus(f3, isoval, min, max, stepsize, vertexQueue, normalQueue, "function3.ply");
			}
		});

		t1.detach();
	}

	GLuint programID = LoadShaders( "DiffuseShader.vertexshader", "DiffuseShader.fragmentshader" );
	GLuint MVPID = glGetUniformLocation(programID, "MVP");
	GLuint VID = glGetUniformLocation(programID, "V");
	GLuint LightDirID = glGetUniformLocation(programID, "LightDirection_worldspace");
	GLuint colorID = glGetUniformLocation(programID, "modelColor");
	
	glm::vec3 lightDir(10.0f, 10.0f, 10.0f);
	glm::vec4 modelColor(0.0f, 0.8f, 0.8f, 1.0f);

	do {
		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		cameraControlsGlobe(V, {5.0f, 5.0f, 5.0f});

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadMatrixf(&V[0][0]);
		worldaxes.draw();
		
		glColor3f(0.7f, 0.7f, 0.7f);
		glLineWidth(2.0f);
		drawBox(min, max);
		glPopMatrix();

		glUseProgram(programID);
		glm::mat4 MVP = Projection * V * M;
		glUniformMatrix4fv(MVPID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(VID, 1, GL_FALSE, &V[0][0]);
		glUniform3f(LightDirID, lightDir.x, lightDir.y, lightDir.z);
		glUniform4fv(colorID, 1, &modelColor[0]);

		int size = marchingVerts.size()/3;
		if(bonus == 1) {
			int qLen = 0;
			if(vertexQueue.size() > normalQueue.size()) qLen = normalQueue.size();
			else qLen = vertexQueue.size();
			
			int i = 0;
			while(i < qLen) {
				face vface; 
				if(vertexQueue.try_pop(vface)) {
					marchingVerts.emplace_back(vface.v1.x);
					marchingVerts.emplace_back(vface.v1.y);
					marchingVerts.emplace_back(vface.v1.z);
					marchingVerts.emplace_back(vface.v2.x);
					marchingVerts.emplace_back(vface.v2.y);
					marchingVerts.emplace_back(vface.v2.z);
					marchingVerts.emplace_back(vface.v3.x);
					marchingVerts.emplace_back(vface.v3.y);
					marchingVerts.emplace_back(vface.v3.z);
				};
				
				face nface; 
				if(normalQueue.try_pop(nface)) {
					normals.emplace_back(nface.v1.x);
					normals.emplace_back(nface.v1.y);
					normals.emplace_back(nface.v1.z);
					normals.emplace_back(nface.v2.x);
					normals.emplace_back(nface.v2.y);
					normals.emplace_back(nface.v2.z);
					normals.emplace_back(nface.v3.x);
					normals.emplace_back(nface.v3.y);
					normals.emplace_back(nface.v3.z);
				}
				i++;
			}

			if(marchingVerts.size()/3 > normals.size()/3) size = normals.size()/3;
			else size = marchingVerts.size()/3;
		}

		glEnableVertexAttribArray(0);
        glVertexAttribPointer(
            0,
            3,
            GL_FLOAT,
            GL_FALSE,
            3 * sizeof(GL_FLOAT),
            &marchingVerts[0]
        );

        // 2nd attribute buffer : normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(
            1,
            3,
            GL_FLOAT,
            GL_TRUE,
            3 * sizeof(GL_FLOAT),
            &normals[0]
        );

		glDrawArrays(GL_TRIANGLES, 0, size);

		glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
		glUseProgram(0);

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// Close OpenGL window and terminate GLFW
	glfwTerminate();
	return 0;
}
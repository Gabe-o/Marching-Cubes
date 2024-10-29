#ifndef _CAM_CONTROLS_H_
#define _CAM_CONTROLS_H_

#include <glm/glm.hpp>
#include "math.h"

static const double _PI = 2.0*asin(1);

void cameraControlsGlobe(glm::mat4& V, glm::vec3 start) {
    static float radius = sqrt(start.x*start.x + start.y*start.y + start.z*start.z);
    static float theta = atan(start.z/start.x);
    static float phi = acos(start.y/radius);

    double currentTime = glfwGetTime();
    static double lastTime = glfwGetTime();
    float deltaTime = (currentTime - lastTime);
    lastTime = currentTime;

    float dy = 0.0f;
    float speed = 10.0f;
    // Move forward
    if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS) {
        dy -= deltaTime * speed;
    }
    // Move backward
    if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS) {
        dy += deltaTime * speed;
    }

    if (dy != 0.0f) {
        radius += dy;
        if (radius < 0.0f) radius = 0.0001f;
    }
    
    
    static double mouseX, mouseY, clickedMouseX, clickedMouseY; 
    static float startTheta, startPhi;
    double offsetX, offsetY;
    float sensitivity = 0.002f;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    static bool mouseHeld = false;
    if (glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_LEFT ) == GLFW_PRESS) {
        
        if(!mouseHeld) {
            clickedMouseX = mouseX;
            clickedMouseY = mouseY;
            startTheta = theta;
            startPhi = phi;
            mouseHeld = true;
        }
        else {
            offsetX = mouseX - clickedMouseX;
            offsetY = clickedMouseY - mouseY; // reversed since y is from bottom to top

            if(startPhi > 0.0f && startPhi < _PI) {
                theta = startTheta + offsetX * sensitivity;
            }
            else {
                theta = startTheta - offsetX * sensitivity;
            }   
            phi = startPhi + offsetY * sensitivity;

            if(phi > _PI) phi -= 2 * _PI;
            if(phi < 0.0f) phi += 2 * _PI;
            if(theta > _PI) theta -= 2 * _PI;
            if(theta < 0.0f) theta += 2 * _PI;
        }
    }
    if (glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_LEFT ) == GLFW_RELEASE) {
        mouseHeld = false;
    }

    glm::vec3 eye = {radius * cos(theta) * sin(phi), radius * cos(phi), radius * sin(theta) * sin(phi)};
    glm::vec3 up;
    if (phi > 0.0f && phi < _PI) up = glm::vec3(0.0f, 1.0f, 0.0f);
    else up = glm::vec3(0.0f, -1.0f, 0.0f);
    
    V = glm::lookAt(eye, {0.0f, 0.0f, 0.0f}, up);
}

#endif

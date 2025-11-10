#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

#define ERROR ("\x1b[1;31m")

using namespace std;

class Viewer {
private:
    GLFWwindow* window;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    mjModel* m;
    mjData* d;

public:
    Viewer(mjModel* model, mjData* data)
        : m(model), d(data)
    {
        
        if (!glfwInit()) {
            cerr<<ERROR<<"Error: no se pudo inicializar GLFW" << endl;
            exit(1);
        }

        window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", NULL, NULL);
        if (!window) {
            cerr<<ERROR<<"Error: no se pudo crear la ventana GLFW" << endl;
            glfwTerminate();
            exit(1);
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); 


        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);


        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);


        cam.lookat[0] = 0;
        cam.lookat[1] = 0;
        cam.lookat[2] = 1.0;
        cam.distance = 3.0;
        cam.azimuth = 90;
        cam.elevation = -20;
    }

    ~Viewer() {
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void render() {
        
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    bool should_close() {
        return glfwWindowShouldClose(window);
    }
};

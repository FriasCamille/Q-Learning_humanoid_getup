#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

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
        // Inicializar GLFW
        if (!glfwInit()) {
            std::cerr << "Error: no se pudo inicializar GLFW" << std::endl;
            exit(1);
        }

        // Crear ventana
        window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", NULL, NULL);
        if (!window) {
            std::cerr << "Error: no se pudo crear la ventana GLFW" << std::endl;
            glfwTerminate();
            exit(1);
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); // vsync

        // Inicializar estructuras de MuJoCo para visualización
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        // Crear escena y contexto de render
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        // Cámara inicial
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
        // Limpiar buffer
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // Renderizar escena
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // Intercambiar buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    bool should_close() {
        return glfwWindowShouldClose(window);
    }
};

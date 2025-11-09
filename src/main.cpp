#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "include/Q_learning.h"
#include "include/enviroment.h"

int main() {
    // Inicializar GLFW
    if (!glfwInit()) {
        std::cerr << "No se pudo inicializar GLFW" << std::endl;
        return 1;
    }

    // Crear ventana
    GLFWwindow* window = glfwCreateWindow(800, 600, "MuJoCo Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "No se pudo crear la ventana" << std::endl;
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // sincronización vsync

    // Cargar modelo
    char error[1024] = "";
    const char* model_path = "resources/darwin_forces.xml"; // cambia por tu XML
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Error cargando modelo: " << error << std::endl;
        return 1;
    }

    mjData* d = mj_makeData(m);

    // Inicializar estructuras de visualización
    mjvCamera cam;            // cámara virtual
    mjvOption opt;            // opciones de renderizado
    mjvScene scn;             // escena
    mjrContext con;           // contexto de renderizado

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Inicializar escena y contexto
    mjv_makeScene(m,&scn, 1000); // 1000 max geom
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Simulación
        mj_step(m, d);

        // Preparar la escena
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

        // Obtener tamaño de ventana
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // Renderizar
        mjrRect viewport = {0, 0, width, height};
        mjr_render(viewport, &scn, &con);

        // Swap buffers y eventos de ventana
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Limpieza
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

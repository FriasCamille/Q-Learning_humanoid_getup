#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"
#include "include/Robot.h"
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>
#include <algorithm> 

using namespace std;

atomic<bool> running(true);

void handle_sigint(int) 
{
    running = false;
}

vector<float> loadVector(const string& filename) 
{
    ifstream in(filename, ios::binary);
    if(!in.is_open()) {
        return vector<float>();
    }
    size_t size;
    in.read(reinterpret_cast<char*>(&size), sizeof(size));
    
    vector<float> v(size);
    in.read(reinterpret_cast<char*>(v.data()), size * sizeof(float));
    return v;
}

void saveVector(const vector<float>& v, const string& filename) 
{
    ofstream out(filename, ios::binary);
    size_t size = v.size();
    out.write(reinterpret_cast<const char*>(&size), sizeof(size));
    out.write(reinterpret_cast<const char*>(v.data()), size * sizeof(float));
}

int main(int argc, char* argv[]) 
{
    signal(SIGINT, handle_sigint);
    
    YAML::Node config = YAML::LoadFile("resources/q_config.yaml");
    auto param = config["parameters"];
    auto p = param[0];
    double e = p["epsilon"].as<double>();
    const double a = p["learning_rate"].as<double>();
    const double gamma = p["gamma"].as<double>();
    const double d = p["discount"].as<double>();
    
    int succes = 0, steps = 0, episodes = 0, motors_numbers = 0, sensors_numbers = 0;
    double average_reward = 0, episode_reward = 0, total_reward = 0;
    
    const int disc = 3;
    const char * trainment_path = (char*)"trainings/getup_trainment";
    
    const char * model_path = (char*)"resources/darwin_forces.xml";
    bool view = false;
    Viewer * viewer = nullptr;
    
    if(argc > 1)
    {
        if(argc > 2) model_path = (char*)argv[2];
        string arg1 = argv[1];
        if(arg1 == "true" || arg1 == "True" || arg1 == "yes" || arg1 == "y" || arg1 == "1") {
            view = true;
        }
    }
    
    Robot darwin("resources/robot_config.yaml", disc);
    motors_numbers = darwin.get_motors().size();
    sensors_numbers = darwin.get_sensors().size();
    cout<<"sensors:"<<sensors_numbers<<endl;
    cout<<"Motor disc: "<<disc<<endl;
    
    long unsigned int states = (long unsigned int)pow(disc, motors_numbers)*18;
    long unsigned int actions = (long unsigned int)pow(3, motors_numbers);
    
    cout << "Configuración del Robot:" << endl;
    cout << "Motores: " << motors_numbers << endl;
    cout << "Sensores: " << sensors_numbers << endl;
    cout << "Estados posibles: " << states << endl;
    cout << "Acciones posibles: " << actions << endl;
    
    long unsigned int present_state = 0, new_state = 0, action_taken = 0;
    double reward_given = 0;
    bool first = true, colision_flag = false, goal_flag = false, step_flag = false, restart = false;
    
    QLearning getup(states, actions, e, a, gamma, d);
    
    vector<float> loaded_qtable = loadVector(trainment_path);
    cout<<"Cargando tabla..."<<endl;
    if(!loaded_qtable.empty() && loaded_qtable.size() == states * actions) 
    {
        getup.set_table(loaded_qtable);  
        cout << "Q-table cargada exitosamente (" << loaded_qtable.size() << " valores)" << endl;
    } else if (!loaded_qtable.empty()) 
    {
        cout << "Q-table inexistente o de tamaño incorrecto. Esperado: " 
             << states * actions << ", obtenido: " << loaded_qtable.size() << endl;
    }

     Environment env(model_path);
    
    if (view) 
    {
        viewer = new Viewer(env.get_model(), env.get_data());
        cout << "Modo visualización ACTIVADO" << endl;
    }
    
    env.forward();
    
    while(running)
    {
        if (restart)
        {
            episodes++;
            total_reward += episode_reward;
            average_reward = total_reward / episodes;
            
            
            if (episodes % 10 == 0) 
            {
                cout << "\n=== EPISODIO " << episodes << " ===" << endl;
                cout << "Recompensa episodio: " << episode_reward << endl;
                cout << "Recompensa promedio: " << average_reward << endl;
                cout << "Épsilon actual: " << getup.get_epsilon() << endl;
                cout << "Pasos: " << steps << endl;
                cout << "Éxitos totales: " << succes << endl;
                cout << "Tasa de éxito: " << (100.0 * succes / max(1, episodes)) << "%" << endl;
                
                
                if (episodes % 50 == 0) 
                {
                    saveVector(getup.get_table(), trainment_path);  
                    cout << "Q-table guardada como backup" << endl;
                }
            }
            
            if (goal_flag) 
            {
                succes++;
                cout << "¡ÉXITO! El robot se levantó correctamente en el episodio " << episodes << endl;
            }
            
            env.reset();
            env.forward();
            
            double random_angle = ((bool)randomInt(0,1))?-M_PI/2:M_PI/2;
            env.write_robot_position(darwin.get_robot_pos().c_str(), random_angle);
            
            for (int i = 0; i < motors_numbers; i++) 
            {
                env.write_joint_position(darwin.get_motor_r_name(i).c_str(), 0);
                env.write_joint_position(darwin.get_motor_l_name(i).c_str(), 0);
                env.write_joint_force(darwin.get_motor_r_name(i).c_str(), 0);
                env.write_joint_force(darwin.get_motor_l_name(i).c_str(), 0);
            }
            
            first = true;
            steps = 0;
            episode_reward = 0;
            colision_flag = false;
            goal_flag = false;
            step_flag = false;
            
            getup.decay_e();
            
            restart = false;
        }
        
        // Leer estado actual
        for (int i = 0; i < motors_numbers; i++) 
        {
            double motor_angle = env.read_joint_position(darwin.get_motor_r_name(i).c_str());
            darwin.set_motor(i, motor_angle);
        }
        
        // Leer sensores
        double torso_pitch = env.get_body_pitch(darwin.get_sensor_name(0).c_str());
        darwin.set_sensor(0, torso_pitch);
        
        // Actualizar Q-learning si no es el primer paso
        if(!first) 
        {
            new_state = darwin.get_state_index();
            getup.update(present_state, action_taken, reward_given, new_state);
            present_state = new_state;
        }
        else 
        {
            present_state = darwin.get_state_index();
            first = false;
        }
        
        // Selección de acción ε-greedy
        action_taken = getup.e_greedy(present_state);
        darwin.action(action_taken);
        
        // Aplicar fuerzas a los motores
        for (int i = 0; i < motors_numbers; i++) 
        {
            env.write_joint_force(darwin.get_motor_r_name(i).c_str(), darwin.get_motor_position(i));
            env.write_joint_force(darwin.get_motor_l_name(i).c_str(), -darwin.get_motor_position(i));
        }
        steps++;
        
        // Avanzar simulación
        for(int i = 0; i < 200; i++) 
        {
            env.simstep();
            
        }
        
        // Renderizar si está activado
        if(view) 
        {
            viewer->render();
            if(viewer->should_close()) 
            {
                running = false;
                break;
            }
            // Pequeña pausa para visualización
            //this_thread::sleep_for(chrono::milliseconds(1));
        }
        
        double torso_height =env.get_body_height("torso");
        
        double total_collision_force = env.collision_force("r_foot", "ground") + 
                                       env.collision_force("l_foot", "ground");
        
        colision_flag = env.collision("body", "ground");
        double com_err = env.read_COM()[0] - env.read_COM("foot_r")[0];
        reward_given = darwin.reward((env.collision_force("r_foot","ground")>0), colision_flag, torso_pitch, total_collision_force,com_err, torso_height);
        episode_reward += reward_given;
        
        // Verificar condiciones de terminación
        goal_flag = darwin.goal();
        step_flag = (steps > 500);
        restart = (goal_flag || colision_flag || step_flag);
        
        if(goal_flag) {
            cout << "\nDEBUG - Estado objetivo alcanzado:" << endl;
            cout << "Estado actual: {";
            for (size_t i = 0; i < darwin.get_state().size(); i++) 
            {
                cout << darwin.get_state()[i];
                if(i < darwin.get_state().size() - 1) cout << ", ";
            }
            cout << "}" << endl;
            
            cout << "Ángulo IMU: " << torso_pitch << " rad = " 
                 << torso_pitch * 180.0 / M_PI << "°" << endl;
            
            cout << "Fuerza total contacto: " << total_collision_force << " N" << endl;
        }
        
        if(episodes > 10000) 
        {
            cout << "Límite de episodios alcanzado (10000)" << endl;
            break;
        }
    }
    
    // Estadísticas finales
    cout << "\n=== ENTRENAMIENTO FINALIZADO ===" << endl;
    cout << "Episodios totales: " << episodes << endl;
    cout << "Éxitos: " << succes << endl;
    cout << "Tasa de éxito: " << (100.0 * succes / max(1, episodes)) << "%" << endl;
    cout << "Recompensa promedio: " << average_reward << endl;
    cout << "Épsilon final: " << getup.get_epsilon() << endl;
    
    // Guardar Q-table final
    saveVector(getup.get_table(), trainment_path);
    cout << "Q-table final guardada como 'getup_trainment' en la carpeta trainings" << endl;
    
    // Limpieza
    if(view) delete viewer;
    
    return 0;
}
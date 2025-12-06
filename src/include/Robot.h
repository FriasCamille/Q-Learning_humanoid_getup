#include "Component.h"
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>

using namespace std;

class Robot
{
private:
    vector<Component> motors;
    vector<Component> sensors;
    
    vector<int> state;
    vector<int> goal_iterators;
    
    int D;
    double reward_value;
    string robot_pos;
    
    // Nuevas variables para seguimiento
    double previous_action_sum;
    vector<double> previous_motor_positions;
    
    inline void update_state() 
    {
        for(long unsigned int i=0; i< state.size();i++)
        {
            if(i<motors.size())
                state[i]=motors[i].get_iterator();
            else    
                state[i] = sensors[i-motors.size()].get_iterator();
        }
    }
    
    // Función para calcular cambio en acciones
    inline double calculate_action_change(const vector<double>& current_positions)
    {
        if(previous_motor_positions.empty()) return 0.0;
        
        double total_change = 0.0;
        for(size_t i = 0; i < current_positions.size(); i++)
        {
            total_change += abs(current_positions[i] - previous_motor_positions[i]);
        }
        return total_change / current_positions.size();
    }

public:
    inline vector<Component> get_motors(){return motors;}
    inline vector<Component> get_sensors(){return sensors;}
    inline vector<int> get_iterator(){return goal_iterators;}
    inline vector<int> get_state(){return state;}
    inline double get_motor_position(int i){return motors[i].get_present_position();}
    inline double get_sensor_position(int i){return sensors[i].get_present_position();}
    inline string get_motor_r_name(int i){return motors[i].get_name()[0];}
    inline string get_motor_l_name(int i){return motors[i].get_name()[1];}
    inline string get_sensor_name(int i){return sensors[i].get_name()[0];}
    inline string get_robot_pos(){return robot_pos;}
    inline bool goal(){return (state==goal_iterators)?true:false;}
    
    inline void set_motor(int i, double value){
        previous_motor_positions[i] = motors[i].get_present_position();
        motors[i].prox(value);
    }
    inline void set_sensor(int i, double value){sensors[i].prox(value);}
    
    Robot(const char* config_path, int disc):D(disc), previous_action_sum(0.0)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        
        auto config_motor= config["motor"];
        auto config_sensor= config["sensor"];
        robot_pos = config["position"][0]["name"].as<string>();
        
        // Posiciones objetivo más realistas para levantarse
        vector<double> target_positions = {0.0, 0.5, 0.0, 0.3, 0.0}; // shoulder, elbow, hip, knee, ankle
        
        for (long unsigned int i =0; i<config_motor.size();i++)
        {
            Component aux;
            auto c = config_motor[i];
            aux.add_name(c["r_name"].as<string>());
            aux.add_name(c["l_name"].as<string>());
            double u = c["u_limit"].as<double>();
            double l = c["l_limit"].as<double>();
            double step = abs(u-l)/(D-1);
            
            for (int j =0; j<D; j++)
            {
                aux.add_position((l+j*step));
            }
            
            // Usar posición objetivo en lugar de 0.0
            double target_pos = (i < target_positions.size()) ? target_positions[i] : 0.0;
            goal_iterators.push_back(aux.prox(target_pos));
            aux.set_position(target_pos, aux.prox(target_pos));
            motors.push_back(aux);
        }
        
        previous_motor_positions.resize(motors.size(), 0.0);
        
        for (long unsigned int i =0; i<config_sensor.size();i++)
        {
            Component aux;  
            auto c = config_sensor[i];
            aux.add_name(c["name"].as<string>());
            double u = c["u_limit"].as<double>();
            double l = c["l_limit"].as<double>();
            double step = abs(u-l)/(D-1);
            
            for (int j =0; j<D; j++)
            {
                aux.add_position((l+j*step));
            }
            
            // Para sensores, objetivo cerca de 0 (vertical)
            double sensor_target = (i == 1) ? 0.0 : 0.0; // torso sensor target = 0 (vertical)
            goal_iterators.push_back(aux.prox(sensor_target));
            aux.set_position(sensor_target, aux.prox(sensor_target));
            sensors.push_back(aux);
        }
        state.resize(motors.size() + sensors.size());
        update_state();
    }
    
    double reward(bool collision, double angle, double collision_force, double torso_height = 0.0)
    {
        double reward = 0.0;
        
        // 1. Penalización pequeña por paso de tiempo (incentiva eficiencia)
        reward -= 0.1;
        
        // 2. Penalización por colisión fuerte
        if (collision) {
            reward -= 100.0;
            cout << "¡COLISIÓN! Penalización fuerte aplicada" << endl;
        }
        
        // 3. Recompensa por ángulo vertical (pitch cerca de 0)
        // Usar coseno que es máximo en 0 y mínimo en ±π
        double angle_reward = cos(abs(angle));
        reward += 5.0 * angle_reward;
        
        // 4. Recompensa por fuerza de contacto óptima en los pies
        // Rango óptimo: 10-40N (contacto firme pero no excesivo)
        double optimal_force_low = 10.0;
        double optimal_force_high = 40.0;
        
        if (collision_force > 1.0) {  // Si hay contacto significativo
            if (collision_force >= optimal_force_low && collision_force <= optimal_force_high) {
                // Rango óptimo: recompensa máxima
                double force_ratio = (collision_force - optimal_force_low) / (optimal_force_high - optimal_force_low);
                reward += 2.0 * (1.0 - abs(force_ratio - 0.5) * 2.0);  // Pico en el medio
            } else if (collision_force < optimal_force_low) {
                // Contacto muy suave
                reward += 1.0 * (collision_force / optimal_force_low);
            } else {
                // Contacto demasiado fuerte - penalización progresiva
                double excess = collision_force - optimal_force_high;
                reward -= 0.5 * excess / optimal_force_high;
            }
        }
        
        // 5. Recompensa por altura del torso (si se proporciona)
        if (torso_height > 0.0) {
            reward += 2.0 * torso_height;
        }
        
        // 6. Penalización por movimientos bruscos
        vector<double> current_positions;
        for(size_t i = 0; i < motors.size(); i++) {
            current_positions.push_back(motors[i].get_present_position());
        }
        double action_change = calculate_action_change(current_positions);
        reward -= 0.05 * action_change;
        
        // 7. Gran recompensa por alcanzar estado objetivo
        if(goal_iterators == state) {
            reward += 100.0;  // Recompensa positiva grande
            cout << "¡ESTADO OBJETIVO ALCANZADO! +100 recompensa" << endl;
        }
        
        return reward;
    }
    
    long unsigned int get_state_index()
    {
        long unsigned int state_index=0, pot=1, value=0;
        for (long unsigned int i=0; i<state.size();i++)
        {
            value = state[i];
            state_index += value*pot;
            pot *=D;
        }
        return state_index;
    }
    
    void action(int n)
    {
        vector<double> old_positions;
        for(size_t i = 0; i < motors.size(); i++) {
            old_positions.push_back(motors[i].get_present_position());
        }
        
        int value=0, action=0;
        for (long unsigned int i=0; i<motors.size();i++)
        {
            action=(n%3)-1;
            value = motors[i].get_iterator() + action;
            if (value>=0 && value<D)
                motors[i].set_iterator(value);
            n/=3;
        }
        update_state();
        
        // Actualizar posiciones anteriores para cálculo de cambio
        previous_motor_positions = old_positions;
    }
    
    // Nuevo método para obtener todas las posiciones de motores
    vector<double> get_all_motor_positions() {
        vector<double> positions;
        for(auto& motor : motors) {
            positions.push_back(motor.get_present_position());
        }
        return positions;
    }
};
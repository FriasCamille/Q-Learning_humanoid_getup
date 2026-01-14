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
    
    inline void set_motor(int i, double value){motors[i].prox(value);}
    inline void set_sensor(int i, double value){sensors[i].prox(value);}
    
    Robot(const char* config_path, int disc):D(disc)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        
        auto config_motor= config["motor"];
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
            
            
            double target_pos = (i < target_positions.size()) ? target_positions[i] : 0.0;
            goal_iterators.push_back(aux.prox(target_pos));
            aux.set_position(target_pos, aux.prox(target_pos));
            motors.push_back(aux);
        }
        
        vector <double> values = {-2/3*M_PI, -M_PI/2, -M_PI/4, -M_PI/6, -0.5, 0.4, 0.3, 0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, M_PI/6, M_PI/4, M_PI/2, 2/3 *M_PI};
        Component aux;
        aux.add_name("torso");
        for (unsigned long int i=0; i< values.size(); i++)
        {
            aux.add_position(values[i]);
        }
        sensors.push_back(aux);

        state.resize(motors.size() + sensors.size());
        update_state();
    }
    
    double reward(bool ground,bool collision, double angle, double collision_force, double COM_err, double torso_height = 0.0)
    {
        double reward = 0.0;

        reward -= 0.1;

        if (collision) 
        {
            reward -= 100.0;
            cout << "¡COLISIÓN! Penalización fuerte aplicada" << endl;
        }
        
        double angle_reward = cos(abs(angle));
        reward += 5.0 * angle_reward;
        
        reward-=abs(COM_err)*10;

        if (torso_height > 0.0) 
        {
            reward += 2.0 * torso_height;
        }

        if(!ground)
        {
            reward-=5.0;
        }

        if(goal_iterators == state) 
        {
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
    }
    
};
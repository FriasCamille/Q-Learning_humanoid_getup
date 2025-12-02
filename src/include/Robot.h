#include "Component.h"
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ranges>

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
        inline double get_motors_position(int i){return motors[i].get_present_position();}
        inline double get_sensors_position(int i){return sensors[i].get_present_position();}
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
            auto config_sensor= config["sensor"];
            robot_pos = config["position"][0]["name"].as<string>();

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
                goal_iterators.push_back(aux.prox(0.0));
                aux.set_position(0.0,aux.prox(0.0));
                motors.push_back(aux);
            }


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
                goal_iterators.push_back(aux.prox(0.0));
                aux.set_position(0.0,aux.prox(0.0));
                sensors.push_back(aux);
            }
            state.resize(motors.size() + sensors.size());
            update_state(); 
        }

        double reward(bool colision, double colision_force)
        {
            double reward =0; 
            reward --;
            if (colision) reward-=10000;
            reward-=abs(colision_force)*10;
            for (long unsigned int i =0; i<state.size();i++)
                reward-= abs(goal_iterators[i]-state[i])*10;

            if(goal_iterators == state) reward = -reward;

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
            int value=0, action=0;
            for (long unsigned int i=0; i<motors.size();i++)
            {
                action=(n%3)-1;
                value = motors[i].get_iterator() + action;
                if (value>0 && value<D )
                    motors[i].set_iterator(value);
                n/=3;
            }
            update_state();
        }


};
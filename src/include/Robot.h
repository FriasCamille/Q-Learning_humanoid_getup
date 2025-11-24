#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <random>
#include <yaml-cpp/yaml.h>

using namespace std;

template <int N>
struct motor {
    string lname;
    string rname;
    double positions[N];
};

template <int N>
struct sensor {
    string name;
    double positions[N];
};


int randomInt(int a, int b) 
{
    static std::random_device rd;      
    static std::mt19937 gen(rd());     
    std::uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

template <int D>
class Robot
{

    public:

        vector<motor<D>> motors;
        vector<sensor<D>> sensors;
        string robot_pos; 
        int* motor_iterators;
        int* sensor_iterators;
        int* motor_actions;

        Robot(const char* config_path)
        {
            YAML::Node config = YAML::LoadFile(config_path);

            motor<D> aux;
            sensor<D> saux;

            double step;
            
            auto motors_config = config["motor"];
            motor_iterators = new int[motors_config.size()];
            motor_actions = new int[motors_config.size()];
            for (size_t i = 0; i < motors_config.size(); i++)
            {
                auto m = motors_config[i];

                aux.rname = m["r_name"].as<std::string>();
                aux.lname = m["l_name"].as<std::string>();

                double u = m["u_limit"].as<double>();
                double l = m["l_limit"].as<double>();

                step = std::abs(u - l) / D;

                bool zero_flag = true;

                for (int j = 0; j < D; j++)
                {
                    aux.positions[j] = l + j * step;

                    if (aux.positions[j] >= 0 && zero_flag)
                    {
                        aux.positions[j] = 0.0;
                        zero_flag = false;
                    }
                }

                motors.push_back(aux);
            }

            auto sensor_config = config["sensor"];
            sensor_iterators = new int[sensor_config.size()];
            for (size_t i = 0; i < sensor_config.size(); i++)
            {
                auto s = sensor_config[i];

                saux.name = s["name"].as<std::string>();

                double u = s["u_limit"].as<double>();
                double l = s["l_limit"].as<double>();

                step = std::abs(u - l) / D;

                bool zero_flag = true;

                for (int j = 0; j < D; j++)
                {
                    saux.positions[j] = l + j * step;

                    if (saux.positions[j] >= 0 && zero_flag)
                    {
                        saux.positions[j] = 0.0;
                        zero_flag = false;
                    }
                }

                sensors.push_back(saux);
            }

            robot_pos = config["position"][0]["name"].as<std::string>();
        }

        double reward(int colition, int goal)
        {
            double reward=0.0;
            if (M_PI/2 - sensors[1].positions[sensor_iterators[1]]<0) reward -= 10;
            reward-= colition*1000;
            reward--;
            reward += goal*100;
            return reward;
        }

        void update_motor_positions()
        {
            for (int i=0; i<motors.size();i++)
            {
                if(motor_iterators[i]+motor_actions[i]>=0 && motor_iterators[i]+motor_actions[i]<=9) motor_iterators[i]+=motor_actions[i];
            }
        }

        int prox(double valor,  string motor_name) 
        {
            int prox_iterator=0;
            double lista[D];
            auto it = find_if(motors.begin(), motors.end(),[&motor_name](const motor<D>& m){return m.nombre == motor_name;});
            if (it == motors.end()) 
            {
                cerr<<ERROR<<"motor inexistente"<<endl;
                return 1;
            }
            lista = it.positions;
            
            double diff= valor - lista[0];

            for (int i=0; i<D; i++)
            {
                if (valor - lista[i]<diff) prox_iterator =i;
            }
            return prox_iterator;
        }

        int get_state_index()
        {
            int state_index=0;
            int bits =motors.size()+sensors.size();
            for (int i=bits; i>0;i--) 
            {
                (i>bits-motors.size())? state_index+= (motors[i-sensors.size()]*pow(D,i)) : state_index+= (sensors[i-sensors.size()]*pow(D,i));
            }
            return state_index;
        }

        void action(int n)
        {
            for (int i=motors.size();i>0;i--)
            {
                motor_actions[i] = (n % 3)-1;
                n/=3;
            }
        }

        void random_restart()
        {
            for(int i=0; i<motors.size(); i++)
            {
                motor_iterators[i]=randomInt(0, D);
            }
            
        }
};



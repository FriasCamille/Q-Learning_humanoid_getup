#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace std;

template <size_t N>
struct motor {
    string lname;
    string rname;
    const double positions[N];
};

template <size_t N>
struct sensor {
    string name;
    const double positions[N];
};

template <size_t D>
class Robot
{

    public:

        vector<motor<D>> motors;
        vector<sensor<D>> sensors;
        string robot_pos; 
        int* motor_iterators;
        int* sensor_iterators;
        int* action_iterators;

        Robot(const char* config_path)
        {
            YAML::Node config = YAML::LoadFile(config_path);

            motor<D> aux;
            sensor<D> saux;

            double step;
            
            auto motors_config = config["motor"];
            motor_iterators = new int[motors_config.size()];
            action_iterators = new int[motors_config.size()];
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

        // void action_to_angle()
        // {
        //     for (int i=0; i<motor_number;i++)
        //     {
        //         if(motor_iterator[i]+action[i]>=0 && motor_iterator[i]+action[i]<=9) motor_iterator[i]+=action[i];
        //     }
        // }

        // int prox(double valor,  double (&lista)[resolution]) 
        // {
        //     auto it = lower_bound(lista.begin(), lista.end(), valor);

        //     if (it == lista.begin()) 
        //     {
        //         return 0;
        //     } 
        //     else if (it == lista.end()) 
        //     {
        //         return lista.size()-1;
        //     } 
        //     else 
        //     {
        //         double anterior = *(it - 1);
        //         double actual = *it;
        //         return (abs(valor - anterior) <= abs(valor - actual)) ? it - lista.begin() -1 : it - lista.begin();
        //     }
        // }

        // int get_state_index()
        // {
        //     return motor_iterator[0]*pow(resolution,7)+ motor_iterator[1]*pow(resolution,6)+ motor_iterator[2]*pow(resolution,5)+ motor_iterator[3]*pow(resolution,4)+ motor_iterator[4]*pow(resolution,3)+ sensor_iterator[0]w(resolution,2)+ sensor_iterator[1]*pow(resolution,1);
        // }

        // void action(int act_n)
        // {
        //     motor_action[4] = (n % 3) - 1;  n /= 3;
        //     motor_action[3] = (n % 3) - 1;  n /= 3;
        //     motor_action[2] = (n % 3) - 1;  n /= 3;
        //     motor_action[1] = (n % 3) - 1;  n /= 3;
        //     motor_action[0] = (n % 3) - 1;
        // }
};




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

std::atomic<bool> running(true);

void handle_sigint(int) {
    running = false;
}

std::vector<double> loadVector(const std::string& filename) {
    std::ifstream in(filename, std::ios::binary);
    size_t size;
    in.read(reinterpret_cast<char*>(&size), sizeof(size));

    std::vector<double> v(size);
    in.read(reinterpret_cast<char*>(v.data()), size * sizeof(double));
    return v;
}

void saveVector(const vector<float>& v, const string& filename) {
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
    int succes=0, steps=0, episodes=0, motors_numbers=0, sensors_numbers=0;
    double average_reward=0, episode_reward=0, total_reward=0;

    const int disc =9;

    const char * model_path = (char*)"resources/darwin_forces.xml";
    bool view =false;
    Viewer * viewer =nullptr;
    
    if(argc>1)
    {
        if(argc>2)model_path = (char*)argv[2];
        if(string(argv[1])=="true"|| string(argv[1])=="True"||string(argv[1])=="yes"||string(argv[1])=="y") view=true;
    }
    Environment env(model_path);

    if (view) viewer = new Viewer(env.get_model(), env.get_data());  

    Robot darwin("resources/robot_config.yaml",disc);
    motors_numbers = darwin.get_motors().size();
    sensors_numbers = darwin.get_sensors().size();
    long unsigned int states = (long unsigned int)pow(disc,motors_numbers+ sensors_numbers);
    long unsigned int actions = (long unsigned int)pow(3,motors_numbers);

    long unsigned int present_state, new_state, action_taken;
    double reward_given, sensor_value;
    bool first = true, colision_flag = false, goal_flag=false, step_flag = false, restart =false;

    QLearning getup(states, actions, e,a,gamma,d);

    env.simstep();

    //restart darwin pose
    for (int i=0; i<motors_numbers;i++)
    {
        env.write_joint_position(darwin.get_motor_r_name(i).c_str(),0);
        env.write_joint_position(darwin.get_motor_l_name(i).c_str(),0);
    }
    env.write_joint_position(darwin.get_robot_pos().c_str(),((bool)randomInt(0,1))?-M_PI/2:M_PI/2);

    while(running)
    {
        if (restart)
        {
            episodes++;
            total_reward+=episode_reward;
            average_reward = total_reward/episodes;
            if (goal_flag)
            {
                this_thread::sleep_for(std::chrono::seconds(30));
                succes++;
            }  
            cout<<"Episode_reward:_ "<<episode_reward<<"\t Average_reward:_"<<average_reward<<endl;
            env.reset();
            //darwin.random_restart();
            for (int i=0; i<motors_numbers;i++)
            {
                env.write_joint_position(darwin.get_motor_r_name(i).c_str(),0);
                env.write_joint_position(darwin.get_motor_l_name(i).c_str(),0);
            }
            env.write_joint_position(darwin.get_robot_pos().c_str(),((bool)randomInt(0,1))?-M_PI/2:M_PI/2);

            first = true;
            steps =0;
            episode_reward =0;

            colision_flag =false;
            goal_flag = false;
            step_flag = false;
        }

        //read actual state

        for (int i =0; i< motors_numbers; i++)
        {
            double motor_angle = env.read_joint_position(darwin.get_motor_r_name(i).c_str());
            darwin.set_motor(i,motor_angle);
        }
        sensor_value = env.get_imu_vel(darwin.get_sensor_name(0).c_str()).y;
        darwin.set_sensor(0,sensor_value);
        sensor_value = env.get_imu_angles(darwin.get_sensor_name(1).c_str()).y;
        darwin.set_sensor(0,sensor_value);

        //updaate if not the first

        if(!first)
        {
            new_state = darwin.get_state_index();
            //cout<<"New state "<<new_state<<" OLD state "<<present_state<<" action "<<action_taken<<" reward "<<reward_given<<endl;
            getup.update(present_state,action_taken, reward_given, new_state);
            present_state = new_state;
        }
        else
        {
            present_state = darwin.get_state_index();
            first = false;
        }

        // e greedy algorithm
        action_taken = getup.e_greedy(present_state);
        darwin.action(action_taken);

        //move those motors

        for ( int i=0; i<motors_numbers; i++)
        {
            env.write_joint_force(darwin.get_motor_r_name(i).c_str(),darwin.get_motors_position(i));
            env.write_joint_force(darwin.get_motor_l_name(i).c_str(),-darwin.get_motors_position(i));
        }
        for(int i=0; i<200;i++)
            env.simstep();

        //see the reward

        reward_given = darwin.reward(env.collision("head","ground"),env.collision_force("body","ground"));
        episode_reward+=reward_given;
        steps++;
        if (view )
        {
            viewer->render();
            //this_thread::sleep_for(std::chrono::milliseconds(300));
            if(viewer->should_close()) break;
        }

        goal_flag = darwin.goal();
        colision_flag = env.collision("head","ground");
        step_flag = (steps>100);
        restart = (goal_flag||colision_flag||step_flag);

    }
    cout<<"Exitos:_"<<succes<<endl;

    return 0;
}
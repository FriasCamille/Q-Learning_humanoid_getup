
#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"
#include "include/Robot.h"
#include <cmath>

#define endl (cout<<"\033[0m \n")

int main(int argc, char* argv[]) 
{
    YAML::Node config = YAML::LoadFile("resources/q_config.yaml");
    auto param = config["parameters"];
    auto p = param[0];
    double e = p["epsilon"].as<double>();
    const double a = p["learning_rate"].as<double>();
    const double gamma = p["gamma"].as<double>();
    const double d = p["discount"].as<double>();

    const int disc =10;

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

    Robot<disc> darwin("resources/robot_config.yaml");

    int states = (int)pow(d,darwin.motors.size()+ darwin.sensors.size());
    int actions = (int)pow(3,darwin.motors.size());

    int present_state, new_state, action_taken;
    double reward_given;
    bool first = true;

    QLearning getup(states, actions, e,a,gamma,d);

    vector<double> present_motor_angles(darwin.motors.size());
    vector<double> present_imu_value(darwin.sensors.size());

    env.simstep();
    darwin.random_restart();

    for (long unsigned int i=0; i<darwin.motors.size(); i++)
    {
        env.write_joint_position(darwin.motors[i].rname.c_str(),darwin.motors[i].positions[darwin.motor_iterators[i]]);
        env.write_joint_position(darwin.motors[i].lname.c_str(),-darwin.motors[i].positions[darwin.motor_iterators[i]]);
    }
            env.write_robot_position(darwin.robot_pos.c_str(),darwin.sensors[1].positions[darwin.sensor_iterators[1]]); //robot pose joint that means the angle in y of the robot

    while (getup.get_epsilon()>0.01) 
    {
        //reset after fail or goal

        if(env.collision("head", "ground")||darwin.get_state_index()==darwin.goal_state) 
        {
            env.reset();
            darwin.random_restart();
            for (long unsigned int i=0; i<darwin.motors.size(); i++)
            {
                env.write_joint_position(darwin.motors[i].rname.c_str(),darwin.motors[i].positions[darwin.motor_iterators[i]]);
                env.write_joint_position(darwin.motors[i].lname.c_str(),-darwin.motors[i].positions[darwin.motor_iterators[i]]);
            }
            env.write_robot_position(darwin.robot_pos.c_str(),darwin.sensors[1].positions[darwin.sensor_iterators[1]]);
            first = true;
        }

        //read actual state

        for (long unsigned int i=0; i<darwin.motors.size(); i++)
        {
            present_motor_angles[i]= env.read_joint_position(darwin.motors[i].rname.c_str());
            darwin.motor_iterators[i] = darwin.prox_motor(present_motor_angles[i], darwin.motors[i].rname.c_str());
        }

        present_imu_value[0]=env.get_imu_vel(darwin.sensors[0].name.c_str()).y;
        darwin.sensor_iterators[0] = darwin.prox_sensor(present_imu_value[0],darwin.sensors[0].name.c_str());
        present_imu_value[1]=env.get_imu_angles(darwin.sensors[1].name.c_str()).y;
        darwin.sensor_iterators[1] = darwin.prox_sensor(present_imu_value[1],darwin.sensors[1].name.c_str());

        //updaate if not the first

        if(!first)
        {
            new_state = darwin.get_state_index();
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

        for (long unsigned int i=0; i<darwin.motors.size(); i++)
        {
            env.write_joint_force(darwin.motors[i].rname.c_str(),darwin.motors[i].positions[darwin.motor_iterators[i]]);
            env.write_joint_force(darwin.motors[i].lname.c_str(),-darwin.motors[i].positions[darwin.motor_iterators[i]]);
        }
        env.simstep();

        //see the reward

        reward_given = darwin.reward(env.collision("head","ground"), present_state==darwin.goal_state);


        if (view )
        {
            viewer->render();
            if(viewer->should_close()) break;
        }
    }

    

    return 0;
}
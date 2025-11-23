
#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"
#include "include/Robot.h"
#include <cmath>

int randomInt(int a, int b) 
{
    static std::random_device rd;      
    static std::mt19937 gen(rd());     
    std::uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

// void random_restart(Robot robot, Environment env)
// {
//     for(int i=0; i<robot.motor_number; i++)
//     {
//         robot.motor_iterator[i]=randomInt(0, robot.resolution);
//         //env.write_joint_position(robot.motors[i].)
//     }
    
// }

int main(int argc, char* argv[]) 
{
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

    while (true) 
    {
        //Write logic 
        if (view )
        {
            viewer->render();
            if(viewer->should_close()) break;
        }
    }

    

    return 0;
}
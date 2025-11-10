
#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"

int main(int argc, char* argv[]) 
{
    // Crear entorno
    Environment env(argv[1]);

    if (string(argv[2])=="true")
    {
    Viewer viewer(env.get_model(), env.get_data());  
    // Bucle principal
    double i=1.5;
        while (!viewer.should_close()) 
        {
            env.simstep();
            cout<< env.get_imu_angles("imu_quat").y<<endl;
            env.write_joint_force("r_elbow", i);
            viewer.render();
            if(env.read_joint_position("r_elbow")==1.5)env.reset();
        }
    }
    // Bucle principal
    double i=1.5;
    while (true) 
    {
        env.simstep();
        cout<< env.get_imu_angles("imu_quat").y<<endl;
        env.write_joint_force("r_elbow", i);
        if(env.read_joint_position("r_elbow")==1.5)env.reset();
    }

    

    return 0;
}
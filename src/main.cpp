
#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"
#include <cmath>


int m1_iterator =0;
int m2_iterator =0;
int m3_iterator =0;
int m4_iterator =0;
int m5_iterator =0;
int i_iterator =0; 
int w_iterator =0; 

vector<double> m1 = {0, 0.471239, 0.942478, 1.413717, 1.884956, 2.356194, 2.827433, 3.298672, 3.769911, 4.241150, 4.712389};
vector<double> m2 = {0, 0.174533, 0.349066, 0.523599, 0.698132, 0.872665, 1.047198, 1.221730, 1.396263, 1.570796, 1.745329};
vector<double> m3 = {0, 0.226893, 0.453786, 0.680678, 0.907571, 1.134464, 1.361357, 1.588250, 1.815142, 2.042035, 2.268928};
vector<double> m4 = {0, 0.226893, 0.453786, 0.680678, 0.907571, 1.134464, 1.361357, 1.588250, 1.815142, 2.042035, 2.268928};
vector<double> m5 = {0, 0.209440, 0.418879, 0.628319, 0.837758, 1.047198, 1.256637, 1.466077, 1.675516, 1.884956, 2.094395};
vector<double> i  = {0, 0.209440, 0.418879, 0.628319, 0.837758, 1.047198, 1.256637, 1.466077, 1.675516, 1.884956, 2.094395};
vector<double> w  = {0, 0.209440, 0.418879, 0.628319, 0.837758, 1.047198, 1.256637, 1.466077, 1.675516, 1.884956, 2.094395}; 

vector<int>actions={-1,0,1};

double reward(float imu_angle, int colition, int goal)
{
    double reward;
    if (M_PI/2 - imu_angle<0) reward -= 10;
    reward-= colition*1000;
    reward--;
    reward += goal*100;
    return reward;
}

double action_to_angle(int iterator, int action, vector<double> motor)
{
    return motor[iterator+actions[action]];
}

int prox(double valor, const vector<double>& lista) 
{
    auto it = lower_bound(lista.begin(), lista.end(), valor);

    if (it == lista.begin()) 
    {
        return 0;
    } 
    else if (it == lista.end()) 
    {
        return lista.size()-1;
    } 
    else 
    {
        double anterior = *(it - 1);
        double actual = *it;
        return (abs(valor - anterior) <= abs(valor - actual)) ? it - lista.begin() -1 : it - lista.begin();
    }
}


int get_state_index(int t1,int t2,int t3,int t4,int t5,int imu,int wv)
{
    return t1*pow(m1.size(),7)+ t2*pow(m2.size(),6)+ t3*pow(m3.size(),5)+ t4*pow(m4.size(),4)+ t5*pow(m5.size(),3)+ imu*pow(i.size(),2)+ wv*pow(w.size(),1);
}

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
        env.simstep();
        //read and write something for the simulation
        env.write_joint_force("r_hip_pitch", -1.5);
        cout<<env.get_imu_angles("imu_quat").y<<endl;
        if (view )
        {
            viewer->render();
            if(viewer->should_close()) break;
        }
    }

    

    return 0;
}
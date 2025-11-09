#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#define ERROR ("\x1b[1;31m")
#define joint(name) (mj_name2id(m, mjOBJ_JOINT, name))
#define body(name) (mj_name2id(m, mjOBJ_BODY, name))
#define sensor(name) (mj_name2id(m, mjOBJ_SENSOR, name))

using namespace std;

struct IMUData {
    double x;
    double y;
    double z;
};

class enviroment
{
    private:
    
    const char* model;
    char errstr[1024] = "";
    mjModel * m;
    mjData * d;


    public:

        enviroment(const char* m_dir): model(m_dir), m(mj_loadXML(m_dir,NULL, errstr, sizeof(errstr)))
        {
            if (m==NULL)
            {
                cout<<ERROR<<"No se pudo cargar el modelo"<<endl;
                cout<<ERROR<<errstr<<endl;
                exit(1);
            }
            d = mj_makeData(m);
        }

        ~enviroment()
        {
            mj_deleteModel(m);
        }

        void simstep(){mj_step(m,d);}
        void reset(){mj_resetData(m,d);}

        double read_joint_positions(char * joint){return d->qpos[joint(joint)];}
        double read_joint_velocity(char * joint){return d->qvel[joint(joint)];}
        double read_joint_forces(char * joint){return d->ctrl[joint(joint)];}
        
        double write_joint_positions(char * joint){return d->qpos[joint(joint)];}
        double write_joint_velocity(char * joint){return d->qvel[joint(joint)];}
        double write_joint_forces(char * joint){return d->ctrl[joint(joint)];}

        void get_imu_angles(char * imu_name, double * angles)
        {
            if (sensor(imu_name) < 0) 
            {
                cout << ERROR << "No se encontró el sensor imu_quat" << std::endl;
                return;
            }
            const double* q = d->sensordata + sensor(imu_name);
            double w = q[0], x = q[1], y = q[2], z = q[3];
            double R[3][3];
            R[0][0] = 1 - 2 * (y * y + z * z);
            R[0][1] = 2 * (x * y - w * z);
            R[0][2] = 2 * (x * z + w * y);
            R[1][0] = 2 * (x * y + w * z);
            R[1][1] = 1 - 2 * (x * x + z * z);
            R[1][2] = 2 * (y * z - w * x);
            R[2][0] = 2 * (x * z - w * y);
            R[2][1] = 2 * (y * z + w * x);
            R[2][2] = 1 - 2 * (x * x + y * y);

            angles[0] = asin(-R[2][0]);             //roll
            angles[1] = atan2(R[2][1], R[2][2]);    //pitch
            angles[2] = atan2(R[1][0], R[0][0]);    //yaw
        }

        void get_imu_vel(char * imu_name, double * w)
        {
            if (sensor(imu_name) < 0) 
            {
                cout << ERROR << "No se encontró el sensor imu_gyro" << std::endl;
                return;
            }
            int adr = m->sensor_adr[sensor(imu_name)];
            const double* gyro = d->sensordata + adr;
            // [wx,wy,wz]
            w[0] = gyro[0];
            w[1] = gyro[1];
            w[2] = gyro[2];
            
        }

        double time_now(){return d->time;}

};
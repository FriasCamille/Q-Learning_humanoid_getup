#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cmath>

#define ERROR ("\x1b[1;31m")

using namespace std;

struct IMUData {
    double x, y, z;
};

class Environment {
private:
    const char* model;
    char errstr[1024] = "";
    mjModel* m;
    mjData* d;

public:
    Environment(const char* model_path)
        : model(model_path), m(mj_loadXML(model_path, NULL, errstr, sizeof(errstr))) 
    {
        if (!m) {
            cerr << ERROR << "No se pudo cargar el modelo\n" << errstr << endl;
            exit(1);
        }
        d = mj_makeData(m);
    }

    ~Environment() 
    {
        mj_deleteData(d);
        mj_deleteModel(m);
    }

    void simstep() { mj_step(m, d); }
    void reset() { mj_resetData(m, d); mj_forward(m, d);}

    double read_joint_position(const char* jname) const 
    {
        int jid = mj_name2id(m, mjOBJ_JOINT, jname);
        if (jid < 0) { cerr << ERROR << "Joint no encontrado: " << jname << endl; return 0.0; }
        return d->qpos[m->jnt_qposadr[jid]];
    }

    double read_joint_velocity(const char* jname) 
    {
        int jid = mj_name2id(m, mjOBJ_JOINT, jname);
        if (jid < 0) { cout << ERROR << "Joint no encontrado: " << jname << endl; exit(1); }
        return d->qvel[m->jnt_dofadr[jid]];
    }

    double read_joint_force(const char* jname) 
    {
        int act_id = mj_name2id(m, mjOBJ_ACTUATOR, jname);
        if (act_id < 0) { cout << ERROR << "Joint actuator no encontrado: " << jname << endl; exit(1); }
        return d->ctrl[act_id];
    }

    void write_joint_position(const char* jname, double val) const 
    {
        int jid = mj_name2id(m, mjOBJ_JOINT, jname);
        if (jid < 0) { cerr << ERROR << "Joint no encontrado: " << jname << endl; return; }
        d->qpos[m->jnt_qposadr[jid]] = val;
    }

    void write_joint_velocity(const char* jname, double val) 
    {
        int jid = mj_name2id(m, mjOBJ_JOINT, jname);
        if (jid < 0) { cout << ERROR << "Joint no encontrado: " << jname << endl; return; }
        d->qvel[m->jnt_dofadr[jid]] = val;
    }

    void write_joint_force(const char* jname, double val) 
    {
        int act_id = mj_name2id(m, mjOBJ_ACTUATOR, jname);
        if (act_id < 0) { cout << ERROR << "Joint actuator no encontrado: " << jname << endl; return; }
        d->ctrl[act_id] = val;
    }
    
    IMUData get_imu_angles(const char* imu_name) const 
    {
        int sid = mj_name2id(m, mjOBJ_SENSOR, imu_name);
        if (sid < 0) { cerr << ERROR << "No se encontró el sensor: " << imu_name << endl; exit(1); }
        const double* q = d->sensordata + m->sensor_adr[sid];
        double w = q[0], x = q[1], y = q[2], z = q[3];

        IMUData imu;
        imu.x = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)); // roll
        imu.y = asin(2*(w*y - z*x));                     // pitch
        imu.z = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)); // yaw
        return imu;
    }

    IMUData get_imu_vel(const char* imu_name) const 
    {
        int sid = mj_name2id(m, mjOBJ_SENSOR, imu_name);
        if (sid < 0) { cerr << ERROR << "No se encontró el sensor: " << imu_name << endl; exit(1); }
        const double* gyro = d->sensordata + m->sensor_adr[sid];
        
        IMUData imu;
         imu.x = gyro[0]; //wx 
        imu.y = gyro[1]; //wy
        imu.z = gyro[2]; //wz
        return imu;
    }

    double time_now() const { return d->time; }

    mjModel* get_model() const { return m; }
    mjData* get_data() const { return d; }
};

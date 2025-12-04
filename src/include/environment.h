#include <iostream>
#include <mujoco/mujoco.h>

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
    void forward() { mj_forward(m, d); }
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

    void write_robot_position(const char* jname, double val) const 
    {
        int jid = mj_name2id(m, mjOBJ_JOINT, jname);
        if (jid < 0) 
        {
            cerr << ERROR << "ERROR: Joint no valido: " << jname << endl;
            return;
        }

        int adr = m->jnt_qposadr[jid];
        
        d->qpos[adr + 0] = 0.0;
        d->qpos[adr + 1] = 0.0;
        d->qpos[adr + 2] = 0.1;

        double half = val * 0.5;

        d->qpos[adr + 3] = cos(half);   
        d->qpos[adr + 4] = 0.0;         
        d->qpos[adr + 5] = sin(half);   
        d->qpos[adr + 6] = 0.0;         
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
    
    double get_imu_pitch(const char* imu_loc) const 
    {
        int torso_id = mj_name2id(m, mjOBJ_BODY, imu_loc);
         const double* q = d->xquat + 4 * torso_id;   // w,x,y,z

        double Zw[3];
        double z_local[3] = {0, 0, 1};  
        mju_rotVecQuat(Zw, z_local, q);

        double pitch = atan2(-Zw[0], Zw[2]);

        return pitch;
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

bool collision(const char* geom_name, const char* target_name)
{
    int geom_id = mj_name2id(m, mjOBJ_GEOM, geom_name);
    int target_id = mj_name2id(m, mjOBJ_GEOM, target_name);

    if (geom_id == -1 || target_id == -1) {
        cout << "Error: uno de los geoms no existe: " << geom_name << endl;
        return false;
    }

    // Recorremos todos los contactos
    for (int i = 0; i < d->ncon; i++) 
    {
        const mjContact* c = &d->contact[i];

        if ((c->geom1 == geom_id && c->geom2 == target_id) ||
            (c->geom2 == geom_id && c->geom1 == target_id))
        {
            
            mjtNum force[6];
            mj_contactForce(m, d, i, force);
            
            double normal_force = force[2];
            
            // cout << "Fuerza normal = " << normal_force << " N\n";
            if (normal_force >= 20.0)
                return true;
        }
    }

    return false;
}

double collision_force(const char* geom_name, const char* target_name)
{
    int geom_id = mj_name2id(m, mjOBJ_GEOM, geom_name);
    int target_id = mj_name2id(m, mjOBJ_GEOM, target_name);

    if (geom_id == -1 || target_id == -1) {
        cout << "Error: uno de los geoms no existe: " << geom_name << endl;
        exit(1);
    }

    // Recorremos todos los contactos
    for (int i = 0; i < d->ncon; i++) 
    {
        const mjContact* c = &d->contact[i];

        if ((c->geom1 == geom_id && c->geom2 == target_id) ||
            (c->geom2 == geom_id && c->geom1 == target_id))
        {
            
            mjtNum force[6];
            mj_contactForce(m, d, i, force);
            
            double normal_force = force[2];
            
            return normal_force;
        }
    }

    return 0;
}



    double time_now() const { return d->time; }

    mjModel* get_model() const { return m; }
    mjData* get_data() const { return d; }
};

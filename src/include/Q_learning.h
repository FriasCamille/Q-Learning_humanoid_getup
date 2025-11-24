#include <iostream>
#include <vector>
#include <random>

using namespace std;

class QLearning
{
    private:
        const int n_states;
        const int n_actions;
        float epsilon; 
        const float alpha;
        const float gamma;
        const float decay;
        vector<float> Qtable;

        mt19937 gen;
        uniform_real_distribution<float> dis_eps;
        uniform_int_distribution<int> dis_action;
    public:

        QLearning(int n_s, int n_a, float eps, float lr, float g, float d)
        : n_states(n_s),
          n_actions(n_a),
          epsilon(eps),
          alpha(lr),
          gamma(g),
          decay(d),
          Qtable(n_s * n_a, 0.0f),
          gen(std::random_device{}()),
          dis_eps(0.0f, 1.0f),
          dis_action(0, n_actions - 1) {}

        inline float &Q(int s, int a) noexcept {return Qtable[s*n_actions+a];}

        inline int max_Q(int s) const noexcept
        {
            const float *row = &Qtable[s*n_actions];
            float maxv = row[0];
            int i;
            for (i=0; i<n_actions; i++ )
            {
                if(row[i]>maxv) maxv = row[i];
            }
            return i;
        }
        
        inline int e_greedy(int s) noexcept
        {
            if (dis_eps(gen)<epsilon) return dis_action(gen);
            return max_Q(s);
        }

        inline void decay_e() noexcept{epsilon *=decay; if(epsilon<0.01) epsilon = 0.01;}

        inline void update(int s, int a, float R, int s1) noexcept
        {
            float &qsa = Q(s,a);
            qsa+= alpha * (R+gamma*e_greedy(s1) -qsa);
        }

        inline const vector<float>& get_table()const noexcept{return Qtable;}

        inline double get_epsilon(){return epsilon;}

};

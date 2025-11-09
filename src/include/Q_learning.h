#include <iostream>
#include <vector>
using namespace std;

#define Q(s,actions, a) (table[((s)*(actions)) + (a)]) // Acceso: table[si * actions + aj]

class Q_learning
{
    private:
        int states; 
        int actions;       
        float epsilon;
        float learning_rate;
        float gamma;
        vector<float> table;
        int s=1;
        int a=1;

    public:
        Q_learning(int sn, int an, float e, float l, float g):states(sn), actions(an), epsilon(e), learning_rate(l), gamma(g), table(states*actions,0.2f){}

        void update(int s1,float R){Q(s,actions,a) = Q(s,actions, a) + learning_rate*(R + gamma*(max_Q(s1))- Q(s,actions, a));}

        float max_Q(int s1)
        {
            float max_q = 0.0;
            for (int i=0; i<actions;i++)
            {
                if(max_q<Q(s1,actions,i))
                {
                    max_q= Q(s1,actions,i);
                }
            }
            return max_q;
        }

    
};
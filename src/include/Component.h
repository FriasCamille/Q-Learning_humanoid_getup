
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <random>

using namespace std;


inline int randomInt(int a, int b) 
{
    static random_device rd;      
    static mt19937 gen(rd());     
    uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

class Component
{
    private:
        vector<string> names;
        vector<double> positions;
        int iterator;
    public:

        inline void add_name(string name){names.push_back(name);}
        inline void add_position(double pos){positions.push_back(pos);}
        inline void set_position(double pos, int iterator){positions[iterator]=pos;}
        inline void set_iterator(int it){iterator = it;}
        inline void set_random_iterator(){iterator = randomInt(0, positions.size()-1);}
        inline vector<string> get_name(){return names;}
        inline int get_iterator(){return iterator;}
        inline double get_present_position(){return positions[iterator];}

        inline int prox(double valor)
        {
            double diff = abs(valor- positions[0]);
            int prox_iterator=0;

            for (long unsigned int i=0; i<positions.size(); i++)
            {
                if(diff> abs(valor-positions[i]))
                {
                    diff = abs(valor-positions[i]);
                    prox_iterator = i;
                }

            }
            iterator= prox_iterator;
            return iterator;
        }

        void print_positions()
        {
            cout<<names[0]<<":_ {";
            for (long unsigned int i=0; i<positions.size();i++)
                cout<<positions[i]<<",";
            cout<<"}"<<endl;
        }


        
};
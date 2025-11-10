
#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"

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
        if (view )
        {
            viewer->render();
            if(viewer->should_close()) break;
        }
    }

    

    return 0;
}
#include <iostream>
#include <sstream>
#include"file_operation.hpp"

using namespace std;

void GAZE_ORIENTATION_TRACKING::syn_output::read(const string &in){
    ifstream f;
    f.open(in.c_str());
    int i=0;
    while(!f.eof())
    {
        string snode;
        getline(f,snode);
        if (snode.empty()) {
          std::cout << "empty line "<< std::endl;
          break;
        }
        stringstream ssnode;
        ssnode << snode;

        ssnode >> frame[i];
        cout<<frame[i]<<" ";
        ssnode >> yaw[i];
        cout<<yaw[i]<<" ";
        ssnode >> pitch[i];
        cout<<pitch[i]<<" ";
        ssnode >> roll[i];
        cout<<roll[i]<<" ";
        ssnode >> keynote[i];
        cout<<keynote[i]<<" ";
        ssnode >> eyex[i];
        cout<<eyex[i]<<" ";
        ssnode >> eyey[i];
        cout<<eyey[i]<<endl;
        i++;
    }
}

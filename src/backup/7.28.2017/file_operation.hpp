#ifndef FILE_OPERATION_HPP
#define FILE_OPERATION_HPP
#include <fstream>
#include <string>

namespace GAZE_ORIENTATION_TRACKING{

using namespace std;

enum category{
    Frame,Yaw, Pitch, Roll, Keynote, Eyex, Eyey
};

class syn_output{
public:
    syn_output(const string& in){
        read(in);
    }



    void read(const string& in);

    //Raw data
    float get(category cat, int index){
        switch(cat)
        {
        case Frame:return frame[index];
        case Roll:return roll[index];
        case Pitch:return pitch[index];
        case Yaw:return yaw[index];
        case Eyex:return eyex[index];
        case Eyey:return eyey[index];
        }
    }

    char get_Keynote(int index)
    {
        return keynote[index];
    }

    //The camera axies differ from those of the motion sensor. This transfer the sensor data into camera axies
    float synch_get(category cat, int index){
        switch(cat)
        {
        case Frame:return frame[index];
        case Roll:return roll[index];
        case Pitch:return pitch[index];
        case Yaw:return yaw[index];
        case Eyex:return eyex[index];
        case Eyey:return eyey[index];
        }
    }

    //Frame NO. to index
    int frame2ind(int frame_i)
    {
        return frame_i - frame[0];
    }

private:

    //Frame indexes
    float frame[60000];

    //Motion parameters
    float yaw[60000];
    float pitch[60000];
    float roll[60000];

    //Keynote indicating its groundtruth position
    char keynote[60000];

    //Tracking the position of the eye in PIXELS
    float eyex[60000];
    float eyey[60000];
};


}//end namespace GAZE_ORIENTATION_TRACKING


#endif // FILE_OPERATION_HPP


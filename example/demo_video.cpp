// GridMatch.cpp : Defines the entry point for the console application.

//#define USE_GPU 
#include "Header.h"
#include "gms_matcher.h"
#include "file_operation.hpp"
#include<Eigen/Dense>

#define DEGREE2RAD(x) x*CV_PI/180.0

using namespace cv;
std::string intrinsic_filename = "../param/intrinsics.yml";
std::string setname[3]={
    "/home/cy/Documents/2017_summer/gaze_tracking/set01/05-29-13_h05_m36_s36_raw_scene_wAudio.mov",
    "/home/cy/Documents/2017_summer/gaze_tracking/set02/01-03-12_h10_m55_s46_raw_scene_wAudio.mov",
    "/home/cy/Documents/2017_summer/gaze_tracking/set03/01-16-14_h02_m39_s13_Parking2_scene_wAudio.mov"
};



int main(int argc, char ** argv)
{
    string videofile;
    if(argc<2){
        videofile = setname[2];
    }
    else
    {
        videofile = argv[1];
    }
    undistorter und(intrinsic_filename);
    Mat Oc = Mat::eye(3, 3, CV_64F), M1 = und.get_M1_h();
    cout<<M1<<endl;
    Mat img_pre, img_post;
    VideoCapture v(videofile);

    syn_output motion("../data/output.txt");
    int start_frame = 24833;
    v.set(CV_CAP_PROP_POS_FRAMES, start_frame);

    //Coordinates transform (yaw pitch roll)_gyro = (-pitch  yaw roll)_camera
    cv::Vec3f Initial(DEGREE2RAD(-4.5 ), -DEGREE2RAD(0),DEGREE2RAD(11.7));
    Oc = euler2dcm(Initial);
    v >> img_pre;
    while(v.get(CV_CAP_PROP_POS_FRAMES) <25018){
//        cout<<"\n Frame "<<v.get(CV_CAP_PROP_POS_FRAMES)<<endl;
        cout<<v.get(CV_CAP_PROP_POS_FRAMES)<<" ";
        v >> img_post;
        Oc = Rotation_cal(img_pre, img_post, M1) * Oc;
        dcm2euler_m(Oc);
        motion.print_frame_ori(v.get(CV_CAP_PROP_POS_FRAMES)-1);
        img_pre = img_post.clone();
        //v.set(CV_CAP_PROP_POS_FRAMES,v.get(CV_CAP_PROP_POS_FRAMES)+3);
    }
	return 0;
}




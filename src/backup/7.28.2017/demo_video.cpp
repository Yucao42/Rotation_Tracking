// GridMatch.cpp : Defines the entry point for the console application.

//#define USE_GPU 
#include "Header.h"
#include "gms_matcher.h"
#include<thread>
#include<Eigen/Dense>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>

//#define UNDISTORT(x,y) remap(x, y, map1, map2, INTER_LINEAR)
#define UNDISTORT(x,y) y=x
#define DEGREE2RAD(x) x*CV_PI/180.0

using namespace cv::sfm;
using namespace cv;
void GmsMatch(Mat &img1,Mat &img2);

std::string intrinsic_filename = "param/intrinsics_m.22.yml";
//Set 1 2 3
std::string setname[3]={
    "/home/cy/Documents/2017_summer/gaze_tracking/set01/05-29-13_h05_m36_s36_raw_scene_wAudio.mov",
    "/home/cy/Documents/2017_summer/gaze_tracking/set02/01-03-12_h10_m55_s46_raw_scene_wAudio.mov",
    "/home/cy/Documents/2017_summer/gaze_tracking/set03/01-16-14_h02_m39_s13_Parking2_scene_wAudio.mov"
};
Mat M1, D1, M2, map1, map2;
Size img_size;


Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<double>(0,0), cvMat3.at<double>(0,1), cvMat3.at<double>(0,2),
         cvMat3.at<double>(1,0), cvMat3.at<double>(1,1), cvMat3.at<double>(1,2),
         cvMat3.at<double>(2,0), cvMat3.at<double>(2,1), cvMat3.at<double>(2,2);

    return M;
}

// Calculates rotation matrix given euler angles.
//The euler angles are given in the order of yaw pitch & roll. Z Y X respectively.
//Z-Y-X
Mat euler2dcm(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[2]),   sin(theta[2]),
               0,       -sin(theta[2]),   cos(theta[2])
               );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      -sin(theta[1]),
               0,               1,      0,
               sin(theta[1]),   0,      cos(theta[1])
               );

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[0]),    sin(theta[0]),      0,
               -sin(theta[0]),    cos(theta[0]),       0,
               0,               0,                  1);


    // Combined rotation matrix
    Mat R = R_x * R_y * R_z;

    return R;
}

void dcm2euler(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    double z, y, x;//yaw pitch roll
    double r11=eigMat(0,1), r12=eigMat(0,0), r21=-eigMat(0,2), r31=eigMat(1,2), r32=eigMat(2,2);


    z = atan2(r11, r12) * 180.00 / CV_PI;
    y = asin (r21)* 180.00 / CV_PI;
    x = atan2(r31, r32)* 180.00 / CV_PI;

        cout<<"Yaw angle: "<<z<<", Pitch angle: "<<y<<", Roll angle: "<<x<<"."<<endl;
}

//In motion axies
void dcm2euler_m(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    double z, y, x;//yaw pitch roll
    double r11=eigMat(0,1), r12=eigMat(0,0), r21=-eigMat(0,2), r31=eigMat(1,2), r32=eigMat(2,2);


    z = atan2(r11, r12) * 180.00 / CV_PI;
    y = asin (r21)* 180.00 / CV_PI;
    x = atan2(r31, r32)* 180.00 / CV_PI;

        cout<<"Yaw angle: "<<y<<", Pitch angle: "<<-z<<", Roll angle: "<<x<<"."<<endl;
}

//Converts DCM to a unique rotation angle around one 3D axies.
double dcm2unirotang(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q1(eigMat);
    q1.normalize();
    double angle = acos(q1.w())*360/CV_PI;
    return angle;
}

void rectify()
{
    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        M2 = getOptimalNewCameraMatrix(M1, D1, img_size, 1, img_size, 0);
        initUndistortRectifyMap(M1, D1, Mat(), M2, img_size, CV_16SC2, map1, map2);
        cout<<"Success!"<<endl;
        cout<<D1<<endl;
    }

}




int main(int argc, char ** argv)
{
#ifdef USE_GPU
	int flag = cuda::getCudaEnabledDeviceCount();
	if (flag != 0){ cuda::setDevice(0); }
#endif // USE_GPU

    string videofile;
    if(argc<2){
        videofile = setname[2];
   }
    else
    {
        videofile = argv[1];
    }

    Mat Oc = Mat::eye(3, 3, CV_64F);
    Mat img_pre, img_post, img1r, img2r;
    VideoCapture v(videofile);
    v.set(CV_CAP_PROP_POS_FRAMES, 24833);

//Coordinates transform (yaw pitch roll)_gyro = (-pitch  yaw roll)_camera
    cv::Vec3f Initial(DEGREE2RAD(-4.5 ), -DEGREE2RAD(0),DEGREE2RAD(11.7));
    Oc = euler2dcm(Initial);
    cout<<Oc<<endl;
    dcm2euler(Oc);
    cout<<"Frame rate: "<<v.get(CV_CAP_PROP_FPS)<<endl;
    v >> img_pre;
    img_size = img_pre.size();
    rectify();
    remap(img_pre, img1r, map1, map2, INTER_LINEAR);
    while(v.get(CV_CAP_PROP_POS_FRAMES) <25018){
        cout<<"\n Frame "<<v.get(CV_CAP_PROP_POS_FRAMES)<<endl;
        v >> img_post;
        Oc = runImagePair(img_pre, img_post, M1) * Oc;
//        cout<<"\nCurrent orientation: "<<endl;
        dcm2euler_m(Oc);
        img_pre = img_post.clone();
        //v.set(CV_CAP_PROP_POS_FRAMES,v.get(CV_CAP_PROP_POS_FRAMES)+3);
    }
	return 0;
}



Mat adjustment(vector<KeyPoint>& kp1,vector<KeyPoint>& kp2,vector<DMatch> &inlier, vector<Point2f> &l, vector<Point2f>& r,Mat& mask)
{
    //Extract the correspondence points
    vector<Point2f> pre,next;
    for(size_t i = 0; i < inlier.size(); i++)
    {
        Point2f left = kp1[inlier[i].queryIdx].pt;
        Point2f right = kp2[inlier[i].trainIdx].pt;
        l.push_back(left);
        r.push_back(right);
    }
    pre = l;
    next = r;
    Mat F = Mat(3, 3, CV_32FC1);
    double ppp = 110;
    F = findFundamentalMat(pre, next, mask, FM_RANSAC, 0.51, 0.999);
//    cout<<"sucess"<<endl;
    return F;
}


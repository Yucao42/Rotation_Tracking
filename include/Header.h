// Yu Cao 7/28/2017
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <map>
#include<Eigen/Dense>

using namespace std;
using namespace cv;

class undistorter{
public:

    //Construction from image calibration parameter file
    undistorter(const string& calib_file){
        read_calibfile(calib_file);
    }

    //Read calib file
    void read_calibfile(const string& in){
        FileStorage fs(in, FileStorage::READ);
        fs["M1"] >> M1;
        fs["D1"] >> D1;
    }

    void undistort_map(const Mat&img){
        Size img_size = img.size();
        M2 = getOptimalNewCameraMatrix(M1, D1, img_size, 1, img_size, 0);
        initUndistortRectifyMap(M1, D1, Mat(), M2, img_size, CV_16SC2, map1, map2);
    }

    Mat get_M1(){return M1;}

    //Half resolution, half matrix
    Mat get_M1_h(){
        Mat m_ = M1.clone()/2;
        m_.at<double>(2,2) = 1;
        return m_;
    }

    //Undistortion
    void undistort_mono(Mat & img,Mat & img_und){
        remap(img, img_und, map1, map2, INTER_LINEAR);
    }

private:

    //M1 intrinsics matrix, D1 distortion vector, M2 new camera matrix.
    Mat M1, D1, M2;

    //Undistort map
    Mat map1, map2;

};

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

// Calculates rotation matrix given euler angles.
//The euler angles are given in the order of yaw pitch & roll. Z Y X respectively.
//Z-Y-X
Mat euler2dcm(Vec3f &theta);

void dcm2euler(const cv::Mat &M);


//In motion axies
void dcm2euler_m(const cv::Mat &M);


//Converts DCM to a unique rotation angle around one 3D axies.
double dcm2unirotang(const cv::Mat &M);

//Opencv Keypoints vector to Point2f
void Keypoints2Points2f(vector<KeyPoint>& kp1,vector<KeyPoint>& kp2,vector<DMatch> &inlier, vector<Point2f> &l, vector<Point2f>& r);

//Image matching, find Fundamental matrix and decompose it to get rotation.
cv::Mat Rotation_cal(Mat &img1, Mat &img2, Mat &M1);


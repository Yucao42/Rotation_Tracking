#include "Header.h"

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

//        cout<<"Yaw angle: "<<y<<", Pitch angle: "<<-z<<", Roll angle: "<<x<<"."<<endl;
        cout<<"   "<<y<<"   "<<-z<<"   "<<x<<"."<<endl;
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

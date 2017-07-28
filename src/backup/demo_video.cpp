// GridMatch.cpp : Defines the entry point for the console application.

//#define USE_GPU 
#include "Header.h"
#include "gms_matcher.h"
#include<thread>
#include<Eigen/Dense>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>

#define UNDISTORT(x,y) remap(x, y, map1, map2, INTER_LINEAR)

using namespace cv::sfm;
using namespace cv;
void GmsMatch(Mat &img1,Mat &img2);

std::string intrinsic_filename = "param/intrinsics_m.33.yml";
//std::string intrinsic_filename = "param/intrinsics_m.33.yml";
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

////Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
//void dcm2euler(const cv::Mat &M)
//{
//    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
//    Eigen::Quaterniond q1(eigMat);
//        double heading, attitude, bank, angle;
//        double &yaw = heading,&pitch = attitude, &roll = bank;
//        double test = q1.x()*q1.y() + q1.z()*q1.w();
//        if (test > 0.499) { // singularity at north pole
//            heading = 2 * atan2(q1.x(),q1.w()) *180/CV_PI;
//            attitude = 90;
//            bank = 0;
//            cout<<"Yaw angle: "<<yaw<<", Pitch angle: "<<pitch<<", Roll angle: "<<roll<<"."<<endl;
//            return;
//        }
//        if (test < -0.499) { // singularity at south pole
//            heading = -2 * atan2(q1.x(),q1.w())*180/CV_PI;
//            attitude = - CV_PI/2*180/CV_PI;
//            bank = 0;
//            cout<<"Yaw angle: "<<yaw<<", Pitch angle: "<<pitch<<", Roll angle: "<<roll<<"."<<endl;
//            return;
//        }
//        double sqx = q1.x()*q1.x();
//        double sqy = q1.y()*q1.y();
//        double sqz = q1.z()*q1.z();
//        heading = atan2(2*q1.y()*q1.w()-2*q1.x()*q1.z() , 1 - 2*sqy - 2*sqz)*180/CV_PI;
//        attitude = asin(2*test)*180/CV_PI;
//        bank = atan2(2*q1.x()*q1.w()-2*q1.y()*q1.z() , 1 - 2*sqx - 2*sqz)*180/CV_PI;
//        angle = acos(q1.w())*360/CV_PI;
//        cout<<"Yaw angle: "<<yaw<<", Pitch angle: "<<pitch<<", Roll angle: "<<roll<<"."<<endl;
//        cout<<"Rotation axies: "<<cv::Vec3d(q1.x(),q1.y(),q1.z())<<", rotation angle: "<<angle<<"."<<endl;

//}

//Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
void dcm2euler(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q1(eigMat);

    double angle, z, y, x;//yaw pitch roll
    double r11=eigMat(0,1), r12=eigMat(0,0), r21=-eigMat(0,2), r31=eigMat(1,2), r32=eigMat(2,2);


    z = atan2(r11, r12) * 180 / CV_PI;
    y = asin (r21)* 180 / CV_PI;
    x = atan2(r31, r32)* 180 / CV_PI;

        cout<<"Yaw angle: "<<z<<", Pitch angle: "<<y<<", Roll angle: "<<x<<"."<<endl;
        }

double dcm2unirotang(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q1(eigMat);
    q1.normalize();
    double angle = acos(q1.w())*360/CV_PI;
//    cout<<"Rotation axies: "<<cv::Vec3d(q1.x(),q1.y(),q1.z())<<", rotation angle: "<<angle<<"."<<endl;
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
//        cout<<map1<<endl;
//        cout<<map2<<endl;
        cout<<D1<<endl;
        //remap(img1, img1r, map1, map2, INTER_LINEAR);
    }

}



void runImagePair( Mat &img1,Mat &img2)
{
//rectify(img1,img2 );
	GmsMatch(img1, img2);
}


int main(int argc, char ** argv)
{
#ifdef USE_GPU
	int flag = cuda::getCudaEnabledDeviceCount();
	if (flag != 0){ cuda::setDevice(0); }
#endif // USE_GPU

    string videofile;
    if(argc<2){
        videofile = "/home/cy/Documents/2017_summer/gaze_tracking/set03/01-16-14_h02_m39_s13_Parking2_scene_wAudio.mov";
//        videofile = "/home/cy/Documents/2017_summer/gaze_tracking/set01/05-29-13_h05_m36_s36_raw_scene_wAudio.mov";
    }
    else
    {
        videofile = argv[1];
    }

    Mat img_pre, img_post, img1r, img2r;
    VideoCapture v(videofile);
    v.set(CV_CAP_PROP_POS_FRAMES, 15000);
    cout<<"Frame rate: "<<v.get(CV_CAP_PROP_FPS)<<endl;
    v >> img_pre;
    img_size = img_pre.size();
    rectify();
    remap(img_pre, img1r, map1, map2, INTER_LINEAR);
    imshow("rectified",img1r);
    imshow("original",img_pre);

//    Mat test = imread("/home/cy/Documents/2017_summer/gaze_tracking/checkrboard_views_eyetrackingcamera/view001.png");
//    remap(test, img2r, map1, map2, INTER_LINEAR);
//    imshow("rectified test",img2r);
    waitKey();
    int count =0;
    while(++count){
        v >> img_pre;
        v.set(CV_CAP_PROP_POS_FRAMES, v.get(CV_CAP_PROP_POS_FRAMES)+50);
        v >> img_post;

        imshow("difference1",img_pre-img_post);
        UNDISTORT(img_pre, img1r);
        UNDISTORT(img_post, img2r);
        imshow("difference",img1r-img2r);
//        imshow("now",img1r);imshow("after",img2r);
        //if(count%6 == 0)
        runImagePair(img1r, img2r);
//        img1r = img2r;
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
//    status.reserve(l.size());
    Mat F = Mat(3, 3, CV_32FC1);

    double ppp = 110;

    F = findFundamentalMat(pre, next, mask, FM_RANSAC, 0.51, 0.999);
//    cout<<mask<<endl;
//    cout<<mask.size()<<endl;
//    while (ppp > 5)
//    {
//        vector<Point2f> F2_prepoint, F2_nextpoint;
//        F2_prepoint.clear();
//        F2_nextpoint.clear();
//        ppp = 0;
//        F = findFundamentalMat(pre, next, mask, FM_RANSAC, 1, 0.999);
//        //cout << F << endl;
//        //computeCorrespondEpilines(F_prepoint,1,F,L);
//        for (int i = 0; i < mask.rows; i++)
//        {
//            if (mask.at<uchar>(i, 0) == 0);
//            else
//            {
//                ///circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
//                double A = F.at<double>(0, 0)*l[i].x + F.at<double>(0, 1)*l[i].y + F.at<double>(0, 2);
//                double B = F.at<double>(1, 0)*l[i].x + F.at<double>(1, 1)*l[i].y + F.at<double>(1, 2);
//                double C = F.at<double>(2, 0)*l[i].x + F.at<double>(2, 1)*l[i].y + F.at<double>(2, 2);
//                double dd = fabs(A*r[i].x + B*r[i].y + C) / sqrt(A*A + B*B);
//                cout << "------:" << dd << "   " << l[i].x << "   " << l[i].y << endl;
//                //cout << "A:  " << A << "   B: " << B << "   C:  " << C << endl;
//                ppp += dd;
//                if (dd > 1)
//                {
////                    circle(pre_frame, F_prepoint[i], 6, Scalar(255, 0, 0), 3);
//                }
//                else
//                {
//                    F2_prepoint.push_back(l[i]);
//                    F2_nextpoint.push_back(r[i]);
//                }
//            }
//        }

//        pre = F2_prepoint;
//        next = F2_nextpoint;
//        cout << "--------------       " << ppp << "      ---------------" << endl;
//    }
    cout<<F<<endl;
    cout<<"sucess"<<endl;
    return F;

}


void GmsMatch(Mat &img1,Mat &img2){
	vector<KeyPoint> kp1, kp2;
	Mat d1, d2;
	vector<DMatch> matches_all, matches_gms;

    Ptr<ORB> orb = ORB::create(500);
	orb->setFastThreshold(0);
	orb->detectAndCompute(img1, Mat(), kp1, d1);
	orb->detectAndCompute(img2, Mat(), kp2, d2);

	BFMatcher matcher(NORM_HAMMING);
	matcher.match(d1, d2, matches_all);

	// GMS filter
	int num_inliers = 0;
	std::vector<bool> vbInliers;
	gms_matcher gms(kp1,img1.size(), kp2,img2.size(), matches_all);
	num_inliers = gms.GetInlierMask(vbInliers, false, true);

	cout << "Get total " << num_inliers << " matches." << endl;

	// draw matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(matches_all[i]);
		}
	}

    //Find Fundamental matrix
    Mat mask;
    vector<Point2f> l,r;
    Mat F = adjustment(kp1,kp2,matches_gms,l,r,mask);
    Mat E = M2.t() * F * M2;

    //Perfrom SVD on E
    SVD decomp = SVD(E);

    //U
    Mat U = decomp.u;

    //S
    Mat S(3, 3, CV_64F, Scalar(0));
    S.at<double>(0, 0) = decomp.w.at<double>(0, 0);
    S.at<double>(1, 1) = decomp.w.at<double>(0, 1);
    S.at<double>(2, 2) = decomp.w.at<double>(0, 2);

    //V
    Mat V = decomp.vt; //Needs to be decomp.vt.t(); (transpose once more)

    //W
    Mat W(3, 3, CV_64F, Scalar(0));
    W.at<double>(0, 1) = -1;
    W.at<double>(1, 0) = 1;
    W.at<double>(2, 2) = 1;

    Mat R,t;
    Mat R1 = U * W * V.t();
    Mat R2 = U * W.t() * V.t();
    if (determinant(R1) < 0 ) R1 = -R1;
    if (determinant(R2) < 0 ) R2 = -R2;
//    cout << "computed rotation: " << endl;
//    cout << R1 << endl;
//    cout << R2 << endl;
//    dcm2euler(R1);
//    dcm2euler(R2);

    recoverPose(E,l,r,M2,R,t);
    dcm2euler(R);
//    cout << "real rotation:" << endl;
//    Mat rot;
//    Rodrigues(images[1].rvec - images[0].rvec, rot); //Difference between known rotations
//    cout << rot << endl;

	Mat show = DrawInlier(img1, img2, kp1, kp2, matches_gms, 1);
    imshow("show", show);
    waitKey(0);
}



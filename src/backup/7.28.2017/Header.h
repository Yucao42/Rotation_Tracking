// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <map>
using namespace std;
using namespace cv;


void Keypoints2Points2f(vector<KeyPoint>& kp1,vector<KeyPoint>& kp2,vector<DMatch> &inlier, vector<Point2f> &l, vector<Point2f>& r);


cv::Mat runImagePair(Mat &img1, Mat &img2, Mat &M1);

#ifdef USE_GPU
	#include <opencv2/cudafeatures2d.hpp>
	using cuda::GpuMat;
#endif


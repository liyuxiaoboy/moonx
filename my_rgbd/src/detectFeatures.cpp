//
// Created by yx_li on 18-11-12.
//
#include "slamBase.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

int main(int argc, char** argv)
{
    cv::Mat rgb1=cv::imread("testimg.png");
    cv::Mat rgb2=cv::imread("rgb2.png");
    cv::Mat depth1=cv::imread("testdepth.png");
    cv::Mat depth2=cv::imread("depth2.png");

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector =cv::FeatureDetector::create("ORB");
    descriptor =cv::DescriptorExtractor::create("ORB");

    vector<cv::KeyPoint> kp1,kp2;
    detector->detect(rgb1,kp1);
    detector->detect(rgb2,kp2);

    cout<<"key points of two imgs : " << kp1.size()<<","<<kp2.size()<<endl;
    cv::Mat imgShow;
    cv::drawKeypoints(rgb1,kp1,imgShow,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("keypoints",imgShow);
    cv::imwrite("./opt/keypoints.png",imgShow);
    cv::waitKey(0);


}

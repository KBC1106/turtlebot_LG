#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    std::string file_path = ros::package::getPath("opencv_practice") + "/img/";

    cv::Mat img = cv::imread(file_path + "canvas.png", cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(), 0.2, 0.2);
    cv::imshow("source_image", img);
	
    cv::Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);

    imshow("gray_image", gray);
    vector<Vec3f> circles;
    double dp = 1;
    double minDist = 20;
    double param1 = 100;
    double param2 = 1;
    double minRadius = 10;
    double maxRadius = 100;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);

    std_msgs::String msg_info;
    stringstream sss;
    //sss << circles.size();
    sss << circles[0][0];
    msg_info.data = sss.str();
    ROS_INFO("%s", msg_info.data.c_str());

    for (int i=0; i<circles.size();i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        //circle( gray, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
        int radius = c[2];
        circle( img, center, radius, Scalar(255, 0, 255), 0.2, LINE_AA);
    }
    imshow("detected circles", img);
  

    

    //input_image -> hsv
    cv::waitKey();

    return 0;
}

#include <std_msgs/String.h>
#include <sstream>
#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

const std::string OPENCV_WINDOW = "ImageWindow";


class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_ori_;
    image_transport::Publisher image_pub_drawn_;
    ros::Publisher chatter_pub = nh_.advertise<std_msgs::String>("chatter_color", 1000); 
public:
    ImageConverter(ros::NodeHandle& nh)
    {
        // Subscribe to input video feed and publish output video feed
        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe("/camera/rgb/image_raw", 1, 
                 &ImageConverter::imageCb, this);

        image_pub_ori_ = it.advertise("/camera/rgb/image_raw", 1);
        image_pub_drawn_ = it.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    };

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    };

    void publishReadImage(void)
    {
        std::string file_path = ros::package::getPath("test_opencv") + "/img/test_img/index.jpeg";
        cv::Mat color_image = cv::imread(file_path, cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
        image_pub_ori_.publish(msg);
    };
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;

        std::string file_path = ros::package::getPath("opencv_practice") + "/img/makemaskin.png";
        //Mat img = imread(file_path, IMREAD_COLOR);
	imshow("source_image", img);
	

	
        //input_image -> hsv
	
        Mat hsv;
        cvtColor(img ,hsv, CV_BGR2HSV);
	Mat mask;	

	cv::inRange(hsv, Scalar(0,100,0,0), Scalar(10, 255, 255, 0), mask);
	imshow("mask",mask);
/*
        for (int x=0;x<mask.cols;x++){
 		for (int y=0;y<mask.rows;y++){
			if(int(mask.at<uchar>(y,x))==0)mask.at<uchar>(y,x) = 255;
			else mask.at<uchar>(y,x)=0;
		}
	}
*/
	
	Mat img_and = img.clone() ;
	
	for (int y=0;y<img.rows;y++){
		for (int x=0;x<img.cols;x++){
			if(int(mask.at<uchar>(y, x)) == 0){
				img_and.at<Vec3b>(y, x)[0] = 0;
				img_and.at<Vec3b>(y, x)[1] = 0;
				img_and.at<Vec3b>(y, x)[2] = 0;
			}
		}
	}
  	imshow("clone", img_and);
	Mat img_and_gray;
	std::vector<Vec3f> circles;
        cvtColor(img_and,img_and_gray, COLOR_BGR2GRAY);
        imshow("img_and_gray", img_and_gray);
	//平滑化
	GaussianBlur(img_and_gray, img_and_gray, Size(9,7), 8, 6);
        //エッジ検出
	double dp = 1;
	double minDist = 20;
	double param1 = 50;//edgeパラメータ
	double param2 = 20;//投票数しきい値
	double minRadius = 20;
	double maxRadius = 300;
	Mat edge;
	Canny(img_and_gray, edge, param1/2, param1);
	//edge画像表示
	imshow("edge", edge);
	
	HoughCircles(img_and_gray, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);


	std_msgs::String msg_info;
	std::stringstream sss;

	sss <<  circles.size(); 
	if (circles.size() > 0){
		sss << "circle: " << circles[0];
		Mat ball_detected_img = img.clone();
		double t = 3;
		double m = 8;

		int index = 0;		
		for (int i=0;i<circles.size();i++){
			int p_r = circles[index][2];
			int r = circles[i][2];
			if (r > p_r)index = i;
		}
		double radius = circles[index][2];
		circle(ball_detected_img, Point(circles[index][0], circles[index][1]), radius, Scalar(0, 255, 0), t, m); 
		
		imshow("ball_ detected_image",ball_detected_img);
	}
	msg_info.data = sss.str();
	ROS_INFO("%s", msg_info.data.c_str());
	
   };
};

int main(int argc, char** argv)
{
    int count = 0;
    ros::init(argc, argv, "opencv_ros");
    ros::NodeHandle nh;
    ImageConverter ic(nh);



    ros::Rate looprate (5);   // read image at 5Hz
    while (ros::ok())
    {
   
        

        if (cv::waitKey(1) == 'q')
            break;

 //       ic.publishReadImage();
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}

#include "std_msgs/UInt16.h"
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

const std::string OPENCV_WINDOW = "ImageWindow";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_ori_;
    image_transport::Publisher image_pub_drawn_;
    ros::Publisher chatter_pub = nh_.advertise<std_msgs::String>("chatter_color", 1000); 
    // Servo Pub //
    ros::Publisher servo_pub = nh_.advertise<std_msgs::UInt16>("/servo", 10);
    ///////////////
    

public:
    ImageConverter(ros::NodeHandle& nh)
    {
        // Subscribe to input video feed and publish output video feed
        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe("/camPi/image_raw", 1, 
                 &ImageConverter::imageCb, this);

        image_pub_ori_ = it.advertise("/camera/image_raw", 1);
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
        cv::Mat input_image = cv_ptr->image;
        //cv::Mat input_image = cv::imread("~/field.png", cv::IMREAD_COLOR);


        //input_image -> hsv
        cv::Mat hsv_image;
        cv::cvtColor(input_image ,hsv_image, CV_BGR2HSV);
        cv::flip(hsv_image, hsv_image, 1);
        cv::flip(hsv_image, hsv_image, 0);
        cv::imshow("HSV_image", hsv_image);

        //hsv_image -> red,yellow,blue
        int width = hsv_image.cols;
        int height = hsv_image.rows;
        int hue, sat, val;
        cv::Mat red_image = cv::Mat(cv::Size(width, height), CV_8UC1);
        cv::Mat yellow_image = cv::Mat(cv::Size(width, height), CV_8UC1);
        cv::Mat blue_image = cv::Mat(cv::Size(width, height), CV_8UC1);

        for (int y=0;y<height;y++){
            for(int x=0;x<width;x++){
                 hue = hsv_image.at<cv::Vec3b>(y, x)[0];
                 sat = hsv_image.at<cv::Vec3b>(y, x)[1];
                 val = hsv_image.at<cv::Vec3b>(y, x)[2];
                 if((hue<8||hue>168) && sat > 100)
                      red_image.at<unsigned char>(y, x) = 0;
                 else
                      red_image.at<unsigned char>(y, x) = 255;
                 if((hue>20&&hue<40) && (sat>20&&sat<100) && val>50)
                      yellow_image.at<unsigned char>(y, x) = 0;
                 else
                      yellow_image.at<unsigned char>(y, x) = 255;
                 if((hue>97&&hue<117) && sat>100 && val>100)
                      blue_image.at<unsigned char>(y, x) = 0;
                 else
                      blue_image.at<unsigned char>(y, x) = 255;
            }
        }
   
        //red_image,yellow_image,blue_imageの黒いピクセルの個数をカウント
/*
        imshow("red_image",red_image);
        imshow("yellow_image",yellow_image);
        imshow("blue_image",blue_image);
*/

        long int red_image_count=0;
        long int yellow_image_count=0;
        long int blue_image_count=0;
        for (int y=0;y<height;y++){
            for (int x=0;x<width;x++){
                if (red_image.at<unsigned char>(y, x) == 0)red_image_count++;
                if (yellow_image.at<unsigned char>(y, x) == 0)yellow_image_count++;
                if (blue_image.at<unsigned char>(y, x) == 0)blue_image_count++;
            }
        }
/*
       std_msgs::String msg_info;
       std::stringstream sss;
       sss << "red_image_count: " << red_image_count;
       sss << " yellow_image_count: " << yellow_image_count;
       sss << " blue_image_count: " << blue_image_count;

       msg_info.data = sss.str();
       ROS_INFO("%s", msg_info.data.c_str());
*/
       cv::Mat flipped_image;
       cv::flip(cv_ptr->image, flipped_image, 1);
       cv::flip(flipped_image, flipped_image, 0);
       cv::imshow(OPENCV_WINDOW, flipped_image);

       //色をパブリッシュ
       std_msgs::String msg_color;
       std_msgs::UInt16 msg_servo;
       if (red_image_count > 20000){
           ROS_INFO("RED!!!");
           std::stringstream ss;
           ss << "red";
           msg_color.data = ss.str();
           chatter_pub.publish(msg_color);

           //// servo grab ///
	   msg_servo.data = 170;
	   servo_pub.publish(msg_servo);
	   ///////////////////
       }
       if (yellow_image_count > 30000){
           ROS_INFO("YELLOW!!!");
           std::stringstream ss;
           ss << "yellow";
           msg_color.data = ss.str();
           chatter_pub.publish(msg_color);

	   //// servo grab ///
           msg_servo.data = 10;
	   servo_pub.publish(msg_servo);
	   ///////////////////

       }
       if (blue_image_count > 30000){
           ROS_INFO("BLUE!!!");
           std::stringstream ss;
           ss << "blue";
           msg_color.data = ss.str();
           chatter_pub.publish(msg_color);
       }
       

        //結果の描画
       image_pub_drawn_.publish(cv_ptr->toImageMsg());
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

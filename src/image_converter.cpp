#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <img_proc.h>
#include <iostream>

using namespace cv;

// Define CV2 window name
static const std::string OPENCV_WINDOW = "Image window";

// Instantiate
ImageProcessor img_proc;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
            : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        this->image_sub_ = it_.subscribe("/camera/image", 1,
                                   &ImageConverter::imageCb, this);
        this->image_pub_ = it_.advertise("/image_converter/output_video", 1);

        namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Access cv::Mat image from pointer
        Mat cv_img = cv_ptr->image;

        // Process image
        Mat cv_img_procd = img_proc.process_image(cv_img);

        // Coords for perspective transformation (counterclockwise as used by cv::warpPerspective)

        // cv::circle(cv_img, cv::Point(80, 130), 10, CV_RGB(255,0,0)); // Point 0 (Red)
        // cv::circle(cv_img, cv::Point(280, 130), 10, CV_RGB(255,125,0)); // Point 1 (Orange)
        // cv::circle(cv_img, cv::Point(320, 180), 10, CV_RGB(0,255,0)); // Point 2 (Green)
        // cv::circle(cv_img, cv::Point(50, 180), 10, CV_RGB(0,0,255)); // Point 3 (Blue)


        // Update GUI Window
        imshow(OPENCV_WINDOW, cv_img_procd);
        waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
};
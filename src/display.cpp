#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <img_proc.h>
#include <iostream>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

// Instantiate
ImageProcessor img_proc;

class DisplayROSImage
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    DisplayROSImage()
            : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image", 1,
                                   &DisplayROSImage::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        // Define CV2 window name

        namedWindow(OPENCV_WINDOW);
    }

    ~DisplayROSImage()
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
        Mat processed_img = img_proc.process_image(cv_img);

        // Update GUI Window
        imshow(OPENCV_WINDOW, processed_img);
        waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    DisplayROSImage ros_img;
    ros::spin();
    return 0;
}
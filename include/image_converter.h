#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// #ifndef IMAGE_CONVERTER
// #define IMAGE_CONVERTER

class ImageConverter {
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
        cv::Mat cv_img;
    public:
        ImageConverter();
        //~ImageConverter();
        void ImageCallBack(const sensor_msgs::ImageConstPtr &msg);
        cv::Mat& get_img() { return cv_img; };
        void testMe();
};
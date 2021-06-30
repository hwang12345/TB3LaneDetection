#include <../include/image_converter.h>

static const std::string OPENCV_WINDOW = "Image window";

ImageConverter::ImageConverter()
    : it_(nh_)
{
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1, &ImageConverter::ImageCallBack, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::Mat cv_img;
    cv::namedWindow(OPENCV_WINDOW);
    cv_bridge::CvImagePtr cv_ptr;
    int i = 4;

}

//ImageConverter::~ImageConverter()
//{
//    cv::destroyWindow(OPENCV_WINDOW);
//}

void ImageConverter::ImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //cv::Mat cv_img = cv_ptr->image;
        //cv_img = this->cv_img;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    this->cv_img = this->cv_ptr->image;


    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}
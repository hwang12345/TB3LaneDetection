#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <iostream>

#include <img_proc.h>
#include <lane_detection.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

// Instantiate
ImageProcessor img_proc;
Lane lane;

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

        const Size img_size = processed_img.size();
        const int rectangle_width = 120;
        const int rectangle_height = 60;

        // Get histogram of image to determine starting x-coord of sliding window
//        Mat hist;
//        int histSize = 256;
//        float range[] = { 0, 256 }; //the upper boundary is exclusive
//        const float* histRange = { range };
//        calcHist(&processed_img, 1, 0, Mat(), hist, 1, &histSize, &histRange, true);
//        cout << hist;

        // Detect left-hand lane
        vector<Point2f> detected_pts_left = lane.sliding_window(processed_img, Rect(0, img_size.height - rectangle_height, rectangle_width, rectangle_height));

        // Detect right-hand lane
        //vector<Point2f> detected_pts_right = lane.sliding_window(processed_img, Rect(img_size.width * 0.5, img_size.height - rectangle_height, rectangle_width, rectangle_height));

        Mat out_img;
        cvtColor(processed_img, out_img, COLOR_GRAY2BGR);

        for (auto i : detected_pts_left) {
            circle(out_img, i, 2, Scalar(255,0,0), 10);
        }

        vector<Point2f> transformed_pts_left;

        Mat invertedPerspectiveMatrix = img_proc.get_invPerspMatrix();

        perspectiveTransform(detected_pts_left, transformed_pts_left, invertedPerspectiveMatrix);

        for (int i = 0; i < transformed_pts_left.size() - 1; ++i) {
            line(cv_img, transformed_pts_left[i], transformed_pts_left[i+1], Scalar(255,0,0), 10);
        }

//        for (auto i : detected_pts_right) {
//            circle(out_img, i, 2, Scalar(255, 0,0), 10);
//        }

        // Update GUI Window
        imshow(OPENCV_WINDOW, cv_img);
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
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
Lane laneLeft;
Lane laneRight;

class DisplayROSImage {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    DisplayROSImage()
            : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image", 1,
                                   &DisplayROSImage::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        // Define CV2 window name

        namedWindow(OPENCV_WINDOW);
    }

    ~DisplayROSImage() {
        destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
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

        Mat invertedPerspectiveMatrix = img_proc.get_invPerspMatrix();
        vector<Point> allPts;

        // Get histogram of image to determine starting x-coord of sliding window
//        Mat hist;
//        int histSize = 256;
//        float range[] = { 0, 256 }; //the upper boundary is exclusive
//        const float* histRange = { range };
//        calcHist(&processed_img, 1, 0, Mat(), hist, 1, &histSize, &histRange, true);
//        cout << hist;

        // Detect left-hand lane
        vector<Point2f> detected_pts_left = laneLeft.sliding_window(processed_img,
                                                                    Rect(0, img_size.height - rectangle_height,
                                                                         rectangle_width, rectangle_height));

        vector<Point2f> transformed_pts_left;
        perspectiveTransform(detected_pts_left, transformed_pts_left, invertedPerspectiveMatrix);

        for (int i = 0; i < transformed_pts_left.size() - 1; ++i) {
            line(cv_img, transformed_pts_left[i], transformed_pts_left[i + 1], Scalar(255, 0, 0), 10);
            allPts.push_back(Point(transformed_pts_left[i].x, transformed_pts_left[i].y));
        }

        // Detect right-hand lane
        vector<Point2f> detected_pts_right = laneRight.sliding_window(processed_img,
                                                                      Rect(img_size.width - rectangle_width,
                                                                           img_size.height - rectangle_height,
                                                                           rectangle_width, rectangle_height));

        vector<Point2f> transformed_pts_right;
        perspectiveTransform(detected_pts_right, transformed_pts_right, invertedPerspectiveMatrix);

        for (int i = 0; i < transformed_pts_right.size() - 1; ++i) {
            line(cv_img, transformed_pts_right[i], transformed_pts_right[i + 1], Scalar(0, 0, 255), 10);
            allPts.push_back(Point(transformed_pts_right[transformed_pts_right.size() - i - 1].x, transformed_pts_right[transformed_pts_right.size() - i - 1].y));
        }

        // Draw lane overlay by connecting the found points of each lane
        vector<vector<Point>> arr;
        arr.push_back(allPts);
        Mat overlay = Mat::zeros(cv_img.size(), cv_img.type());
        fillPoly(overlay, arr, Scalar(0, 156, 100));
        addWeighted(cv_img, 1, overlay, 0.5, 0, cv_img);

        resize(cv_img, cv_img, Size(700, 500));

        // Update GUI Window
        imshow(OPENCV_WINDOW, cv_img);
        imshow("BEV", processed_img);

        if (waitKey(3) == 27)
            exit(0);

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
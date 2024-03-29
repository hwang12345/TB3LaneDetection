/*
 *
 * Image processing operations for lane-line detection using ROS and OpenCV
 *
 */

#ifndef ROSIMAGE_LANE_DETECTION_H
#define ROSIMAGE_LANE_DETECTION_H

#endif //ROSIMAGE_LANE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace cv_bridge;
using namespace std;

class ImageProcessor {
public:
    Mat transform_img(Mat& img);
    Mat mask_img(Mat& img);
    Mat blur(Mat &img);
    Mat process_image(Mat& img);
    Mat get_invPerspMatrix();

};

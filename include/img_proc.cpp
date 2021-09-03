#include <img_proc.h>

Mat ImageProcessor::process_image(Mat cv_img) {
    if (cv_img.rows > 5 && cv_img.cols > 5)
        cv::circle(cv_img, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    return cv_img;
}
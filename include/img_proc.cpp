#include <img_proc.h>

CvImagePtr ImageProcessor::drawCircle(CvImagePtr cv_ptr) {
    if (cv_ptr->image.rows > 5 && cv_ptr->image.cols > 5)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    return cv_ptr;
}
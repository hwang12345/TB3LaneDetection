#ifndef ROSIMAGE_LANE_DETECTION_H
#define ROSIMAGE_LANE_DETECTION_H

#endif //ROSIMAGE_LANE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

class Lane {
public:
    vector<Point2f> sliding_window();
//
};
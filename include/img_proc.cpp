#include <img_proc.h>

Mat ImageProcessor::transform_img(Mat &img) {

    // Coords for perspective transformation (counterclockwise as used by cv::warpPerspective)

    // cv::circle(cv_img, cv::Point(80, 130), 10, CV_RGB(255,0,0)); // Point 0 (Red)
    // cv::circle(cv_img, cv::Point(280, 130), 10, CV_RGB(255,125,0)); // Point 1 (Orange)
    // cv::circle(cv_img, cv::Point(320, 180), 10, CV_RGB(0,255,0)); // Point 2 (Green)
    // cv::circle(cv_img, cv::Point(50, 180), 10, CV_RGB(0,0,255)); // Point 3 (Blue)

    // Initialize vector for edges and define those
    Point2f srcVertices[4];

    srcVertices[0] = Point(80, 130);
    srcVertices[1] = Point(280, 130);
    srcVertices[2] = Point(320, 180);
    srcVertices[3] = Point(50, 180);

    //Destination vertices. Output is 640 by 480px
    Point2f dstVertices[4];

    dstVertices[0] = Point(0, 0);
    dstVertices[1] = Point(640, 0);
    dstVertices[2] = Point(640, 480);
    dstVertices[3] = Point(0, 480);

    // Compute transform matrix and perform homographic transformation
    Mat perspectiveMatrix = getPerspectiveTransform(srcVertices, dstVertices);
    Mat dst(480, 640, CV_8UC3); //Destination for warped image

    warpPerspective(img, dst, perspectiveMatrix, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}

Mat ImageProcessor::mask_img(Mat &img) {

    cvtColor(img, img, COLOR_RGB2GRAY);

    //Mask yellow and white pixels of image
    Mat maskYellow, maskWhite;

    inRange(img, Scalar(20, 100, 100), Scalar(30, 255, 255), maskYellow);
    inRange(img, Scalar(150, 150, 150), Scalar(255, 255, 255), maskWhite);

    // Combine yellow and white mask and replace in original
    Mat mask, processed;
    bitwise_or(maskYellow, maskWhite, mask);
    bitwise_and(img, mask, processed);

    return processed;
}

Mat ImageProcessor::blur(Mat &img) {
    //Blur to smooth irregularities
    const Size kernelSize = Size(9, 9);
    GaussianBlur(img, img, kernelSize, 0);

    //Fill gaps
    Mat kernel = Mat::ones(15, 15, CV_8U);
    dilate(img, img, kernel);
    erode(img, img, kernel);
    morphologyEx(img, img, MORPH_CLOSE, kernel);

    //Threshold image
    const int thresholdVal = 150;
    threshold(img, img, thresholdVal, 255, THRESH_BINARY);

    return img;
}

Mat ImageProcessor::process_image(Mat &img) {
    Mat processed_img;

    Mat bev_img = this->transform_img(img);
    Mat masked_img = this->mask_img(bev_img);
    Mat blurred_img = this->blur(masked_img);

    processed_img = blurred_img;
    return processed_img;
}

Mat ImageProcessor::get_invPerspMatrix() {
    Point2f srcVertices[4];

    srcVertices[0] = Point(80, 130);
    srcVertices[1] = Point(280, 130);
    srcVertices[2] = Point(320, 180);
    srcVertices[3] = Point(50, 180);

    //Destination vertices. Output is 640 by 480px
    Point2f dstVertices[4];

    dstVertices[0] = Point(0, 0);
    dstVertices[1] = Point(640, 0);
    dstVertices[2] = Point(640, 480);
    dstVertices[3] = Point(0, 480);

    // Compute transform matrix and perform homographic transformation
    Mat perspectiveMatrix = getPerspectiveTransform(srcVertices, dstVertices);
    Mat invertedPerspMatrix;
    invert(perspectiveMatrix, invertedPerspMatrix);

    return invertedPerspMatrix;
}
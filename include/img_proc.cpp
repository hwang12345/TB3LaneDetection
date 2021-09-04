#include <img_proc.h>

Mat ImageProcessor::transform_img(Mat &img) {
    //Prepare things that don't need to be computed on every frame.
    Point2f srcVertices[4];

    // Coords for perspective transformation (counterclockwise as used by cv::warpPerspective)

    // cv::circle(cv_img, cv::Point(80, 130), 10, CV_RGB(255,0,0)); // Point 0 (Red)
    // cv::circle(cv_img, cv::Point(280, 130), 10, CV_RGB(255,125,0)); // Point 1 (Orange)
    // cv::circle(cv_img, cv::Point(320, 180), 10, CV_RGB(0,255,0)); // Point 2 (Green)
    // cv::circle(cv_img, cv::Point(50, 180), 10, CV_RGB(0,0,255)); // Point 3 (Blue)

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

    //Prepare matrix for transform and get the warped image
    Mat perspectiveMatrix = getPerspectiveTransform(srcVertices, dstVertices);
    Mat dst(480, 640, CV_8UC3); //Destination for warped image

    warpPerspective(img, dst, perspectiveMatrix, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}

Mat ImageProcessor::mask_img(Mat &img) {

    cvtColor(img, img, COLOR_RGB2GRAY);

    //Extract yellow and white info
    Mat maskYellow, maskWhite;

    inRange(img, Scalar(20, 100, 100), Scalar(30, 255, 255), maskYellow);
    inRange(img, Scalar(150, 150, 150), Scalar(255, 255, 255), maskWhite);

    Mat mask, processed;
    bitwise_or(maskYellow, maskWhite, mask); //Combine the two masks
    bitwise_and(img, mask, processed); //Extract what matches

    return processed;
}

Mat ImageProcessor::blur(Mat &img) {
    //Blur the image a bit so that gaps are smoother
    const Size kernelSize = Size(9, 9);
    GaussianBlur(img, img, kernelSize, 0);

    //Try to fill the gaps
    Mat kernel = Mat::ones(15, 15, CV_8U);
    dilate(img, img, kernel);
    erode(img, img, kernel);
    morphologyEx(img, img, MORPH_CLOSE, kernel);

    //Keep only what's above 150 value, other is then black
    const int thresholdVal = 150;
    threshold(img, img, thresholdVal, 255, THRESH_BINARY);
    //Might be optimized with adaptive thresh

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
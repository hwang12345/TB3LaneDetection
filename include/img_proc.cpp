#include <img_proc.h>

Mat ImageProcessor::birdseye_view(Mat org_img) {
    //Prepare things that don't need to be computed on every frame.
    Point2f srcVertices[4];

    //Define points that are used for generating bird's eye view. This was done by trial and error. Best to prepare sliders and configure for each use case.
    //    srcVertices[0] = Point(790, 605); //These were another test.
    //    srcVertices[1] = Point(900, 605);
    //    srcVertices[2] = Point(1760, 1030);
    //    srcVertices[3] = Point(150, 1030);

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

    warpPerspective(org_img, dst, perspectiveMatrix, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}

Mat ImageProcessor::process_image(Mat cv_img) {

    Mat birdseye_image = this->birdseye_view(cv_img);

    return birdseye_image;
}
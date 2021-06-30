#include <stdlib.h>
#include <image_converter.h>

using namespace std;
static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
    cout << "\n\n==============START==================\n\n";
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    cv_bridge::CvImagePtr& cv_ptr = ic.cv_ptr;
    cv::Mat& cv_img = ic.cv_img;

    if(cv_ptr)
    {
        cv::imshow(OPENCV_WINDOW, cv_img);
        cv::waitKey(1);
        //cout << "Still in the callback function, dammit!\n";
    }

    //if(cv_ptr)
    //{
        //cv::imshow(OPENCV_WINDOW, img);
        //cv::waitKey(1);

        //cout << "This is the main file!\n";
    //}



    //cv_bridge::CvImagePtr cv_ptr = ic.get_img();
    //cv::Mat& cv_img = ic.get_img();
    //cout << "cv_ptr Image = " << endl << " "  << cv_img << endl << endl;

    //cv::namedWindow(OPENCV_WINDOW);
    //cv::imshow(OPENCV_WINDOW, cv_img);
    //cv::waitKey(1);

    ros::spin();
    return 0;
}



#include "lane_detection.h"

vector <Point2f> Lane::sliding_window(Mat &img, Rect window) {
    vector<Point2f> points;
    const Size img_size = img.size();
    bool break_loop;

    while(true) {

        rectangle(img, window, Scalar(255,100,0), 1, 8);

        float current_x = window.x + (window.width * 0.5f);

        Mat roi = img(window);
        vector<Point2f> locations;

        // Get non-black pixels in window
        findNonZero(roi, locations);

        //std::cout << locations.size();

        float avg_x;

        for (int i = 0; i < locations.size(); ++i) {
            float x = locations[i].x;
            avg_x += window.x + x;
        }
        avg_x = locations.empty() ? current_x : (avg_x / locations.size());

        Point point(avg_x, window.y + (window.height * 0.5f));
        points.push_back(point);

        // Move window up
        window.y -= window.height;

        if (window.y < 0) {
            window.y = 0;
            break_loop = true;
        }

        // Move window sideways
        window.x += (point.x - current_x);



        // Guard against window overflow
        if (window.x < 0)
            window.x = 0;
        if (window.x + window.width >= img_size.width)
            window.x = img_size.width - window.width - 1;

        if (break_loop)
            break;

    }

    return points;

}

#include "lane_detection.h"

vector <Point2f> Lane::sliding_window(Mat &img, Rect window) {
    vector<Point2f> points;
    const Size img_size = img.size();
    bool break_loop;

    while(true) {

        // Draw rectangle on frame
        rectangle(img, window, Scalar(255,100,0), 1, 8);

        Mat roi = img(window);
        vector<Point> locations;

        // imshow("test", roi);
        int count_nz = countNonZero(roi); // If nothing in windows: 356

        // Get non-black pixels in window (pixels with non-zero intensity values, e.g. white pixels)
        // and save them in the vector "locations"
        findNonZero(roi, locations);

        while (count_nz == 356) {
            // Draw rectangle on frame
            rectangle(img, window, Scalar(255,100,0), 1, 8);
            window.x += window.width * 0.5f;
            findNonZero(roi, locations);
            count_nz = countNonZero(roi);
        };

        // Current x coordinate of middle of the window
        float current_x = window.x + (window.width * 0.5f);

        // Initialize average x-position of white pixels to 0
        float avg_x = 0.0f;

        // Iterate through the found locations of white pixels and add up their coordinates
        // in the coordinate system of the sliding window
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

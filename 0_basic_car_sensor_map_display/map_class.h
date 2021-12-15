#ifndef MAP_CLASS
#define MAP_CLASS

#include <iostream>
#include "opencv2/opencv.hpp"

class map_class{
public:
    map_class(int width, int height);
    ~map_class();

    void add_cone(cv::Point2f position);
    cv::Size get_map_size() const;

    std::vector<cv::Point2f> _cone_positions;

private:
    map_class();
    cv::Size _map_size;

    int _width,_height;
};

#endif

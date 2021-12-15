#ifndef VIS_MAP_CLASS
#define VIS_MAP_CLASS

#include "car_class.h"
#include "image_operations.h"
#include "opencv2/opencv.hpp"

// class to draw a map for different object - car, objects, etc
class visualize_map_class{

public:

    // creates an object based on a given map (extracts its size and defines the resolution, ie. how many dots per unit in map)
    visualize_map_class(const map_class &my_map, int resolution);
    ~visualize_map_class();

    // this creates an empty map (with sidebars)
    void init_map(cv::Mat &map_image);

    // plots obstacles in the map
    void plot_obstacles(cv::Mat &map_image, const std::vector<cv::Point2f> &cone_positions);

    // this plots a car given the car parameters and its pose
    void plot_car(cv::Mat &map_image, const car_class &my_car);

    // this plots a scan result, all data is inlcuded in the position object
    void plot_scan(cv::Mat &map_image, car_class my_car, object_position_vector object);

    // creates all points to disaply the car, the 4 corners as well as a direction arrow which points in the car direction
    cv::Mat get_car_mat(const car_class &my_car);

private:
    // need parameters, default constructor not to be used
    visualize_map_class();

    //stores the original mapsize and the requested resolution of the map ie. how many points in display for one point in map
    cv::Size orig_map_size;
    int _resolution;

    // keeps the internal image
    cv::Mat _map_image;

    // coordinate transform from car to opencv (opencv origin in top left -> x stays the same, but the y coordinate is "inverted"), angle considerations are the same
    PPType car_coordinates_to_opencv_coordinates(const PPType &from_position);
};

#endif

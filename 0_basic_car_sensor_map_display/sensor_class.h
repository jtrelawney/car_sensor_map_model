#ifndef SENSOR_CLASS
#define SENSOR_CLASS

#include "map_class.h"
#include "support.h"

#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"

class object_position_vector{

public:
    ~object_position_vector(){};
    object_position_vector(cv::Point2f cone_position, cv::Point2f car_position, double car_alpha, double object_alpha, double object_distance):
        _cone_position(cone_position), _car_position(car_position), _car_alpha(car_alpha), _object_alpha(object_alpha), _object_distance(object_distance){};

    cv::Mat get_pos_mat(){
        cv::Mat pos_mat = ( cv::Mat_<double>(1,2) << (double)_object_alpha, (double)_object_distance );
        return pos_mat;
    };

    cv::Point2f _cone_position;
    cv::Point2f _car_position;
    double _car_alpha;
    double _object_alpha;
    double _object_distance;
private:
    object_position_vector(){};
};

// simulates a sensor with a vision range and max distance
// a reading returns the pose for every object on a map for a given car position (withing range and vision)
class sensor_class{
public:
    sensor_class(double vision_angle, double max_distance);
    ~sensor_class();

    // for each object in the map it simulates a sensor reading and returns the direction fo the object and the distance
    std::vector<object_position_vector> scan(const map_class &my_map, const cv::Point2f &position, const double &alpha);

private:
    sensor_class();
    double _vision_angle, _max_distance;
};

#endif

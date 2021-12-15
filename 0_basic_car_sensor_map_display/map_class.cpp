#include "map_class.h"

map_class::map_class(int width, int height):_width(width),_height(height){
    _map_size = cv::Size(height,width);
}

void map_class::add_cone(cv::Point2f position){
    if ( (position.x>0) && (position.y>0) && (position.x<_width) && (position.y<_height) )
        _cone_positions.push_back(position);
    else std::cout << "cone position" << position << " not valid for map of size " << _width << " * " << _height << std::endl;
}

cv::Size map_class::get_map_size() const {
    return _map_size;
}


map_class::~map_class(){}

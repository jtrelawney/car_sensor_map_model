#include "sensor_class.h"


int main(){


    sensor_class sensor(180, 100);

    map_class m(10,10);
    m.add_cone(cv::Point2f(3,3));
    m.add_cone(cv::Point2f(5,5));

    cv::Point2f position;
    double alpha;
    std::vector<object_position_vector> result;

    // 2 cones on a left diagonal
    position = cv::Point2f(0,0);
    alpha = 0;

    result = sensor.scan(m, position, alpha);
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "car position = " << position << "   alpha = " << alpha << std::endl;
    for (auto p: result){ std::cout << p.get_pos_mat() << std::endl; }
    std::cout << "----------------------------------------------" << std::endl;

    // between the cones looking up at them
    position = cv::Point2f(4,0);
    alpha = 90;

    result = sensor.scan(m, position, alpha);
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "car position = " << position << "   alpha = " << alpha << std::endl;
    for (auto p: result){ std::cout << p.get_pos_mat() << std::endl; }
    std::cout << "----------------------------------------------" << std::endl;

    // between the cones looking up and left
    position = cv::Point2f(6,2);
    alpha = 135;

    result = sensor.scan(m, position, alpha);
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "car position = " << position << "   alpha = " << alpha << std::endl;
    for (auto p: result){ std::cout << p.get_pos_mat() << std::endl; }
    std::cout << "----------------------------------------------" << std::endl;

    // check on range and vision field
    sensor = sensor_class(15, 4);

    // with test case above , one cone is too far left and one cone is in the vision, but too far away
    position = cv::Point2f(4,0);
    alpha = 90;

    result = sensor.scan(m, position, alpha);
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "car position = " << position << "   alpha = " << alpha << std::endl;
    for (auto p: result){ std::cout << p.get_pos_mat() << std::endl; }
    std::cout << "----------------------------------------------" << std::endl;

    return 0;

}
